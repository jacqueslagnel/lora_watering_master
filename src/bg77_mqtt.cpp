/**
 * @file bg77_mqtt.cpp
 * @brief Quectel BG77 LTE/MQTT driver implementation.
 *
 * This file implements BG77 modem control through AT commands, including
 * startup preparation, LTE network profile selection, MQTT open/connect/
 * subscribe/publish operations, inbound MQTT message queuing, and reconnection
 * watchdog behavior.
 */
#include "bg77_mqtt.h"
#include "credentials.h"
#include "watchdog_simple.h"

static const char *BROKER_HOST = CRED_BROKER_HOST;     ///< MQTT broker hostname.
static const uint16_t BROKER_PORT = CRED_BROKER_PORT;  ///< MQTT broker TCP port.
static const char *MQTT_USER = CRED_MQTT_USER;         ///< MQTT username.
static const char *MQTT_PASSWORD = CRED_MQTT_PASSWORD; ///< MQTT password.
static const char *APN_NAME = CRED_APN_NAME;           ///< APN used for LTE packet service.

static char MQTT_CLIENT_ID[32] = "bg77-client-000"; ///< Runtime MQTT client identifier.

static const char *MQTT_TOPIC_SUB = "d/t/cmd"; ///< MQTT topic subscribed for commands.
static const char *MQTT_TOPIC_PUB = "d/t";     ///< Default MQTT publish topic.
static const char *MQTT_TOPIC_ACK = "d/t/ack"; ///< MQTT acknowledgment publish topic.

static const char *PLMN_BOUYGUES = "20820"; ///< Bouygues Telecom PLMN.
static const char *PLMN_ORANGE = "20801";   ///< Orange France PLMN.

static const int ACT_CATM1 = 8; ///< BG77 access technology code for LTE Cat-M1.
static const int ACT_NBIOT = 9; ///< BG77 access technology code for NB-IoT.

static const uint8_t MAX_MQTT_RECOVERY_ATTEMPTS = 3; ///< Maximum MQTT recovery attempts before stronger recovery.
static const uint32_t MODEM_REBOOT_WAIT_MS = 4000;   ///< Wait time after modem reboot operations, in milliseconds.
static const uint32_t MQTT_RETRY_DELAY_MS = 2000;    ///< Delay between immediate MQTT retry attempts, in milliseconds.

static const uint8_t BG77_PWRKEY_IDLE_LEVEL = LOW;    ///< Idle GPIO level for BG77 PWRKEY.
static const uint8_t BG77_PWRKEY_ACTIVE_LEVEL = HIGH; ///< Active GPIO level for BG77 PWRKEY pulse.
static const uint32_t BG77_PWRKEY_PULSE_MS = 900;     ///< BG77 PWRKEY pulse duration, in milliseconds.

static const int NBIOT_PREFERENCE_DB = 5;
static const uint8_t LTE_SIGNAL_SAMPLE_COUNT = 6;
static const uint32_t LTE_SIGNAL_SAMPLE_GAP_MS = 5000UL;

struct LteProfileCandidate
{
    const char *name;
    const char *plmn;
    int act;
    int iotopmode;
    int rsrp;
    bool ok;
};

/**
 * @brief Constructs the BG77 driver and initializes cached state.
 *
 */
Bg77Mqtt::Bg77Mqtt(Print &pcSerial, Uart &modemSerial, uint32_t pwrKeyPin)
    : _pc(pcSerial),
      _modem(modemSerial),
      _pwrKeyPin(pwrKeyPin),
      _networkRegistered(false),
      _packetAttached(false),
      _mqttSocketOpen(false),
      _mqttConnected(false),
      _justConnected(false),
      _lteStatus("UNKNOWN;UNKNOWN"),
      _lastProfileName(""),
      _lastScanSummary("B_NBIOT=-999;O_NBIOT=-999;B_CATM1=-999;O_CATM1=-999"),
      _mqttKeepAliveSec(300),
      _lastMqttOkMs(0),
      _nextReconnectAtMs(0),
      _disconnectedSinceMs(0),
      _mqttReconnectFailures(0),
      _bootPowerCycleDone(false),
      _bootProviderScanDone(false),
      _mqttQueueHead(0),
      _mqttQueueTail(0),
      _mqttQueueCount(0)
{
}

void Bg77Mqtt::begin()
{
    _modem.begin(115200);
    delay(250);

    uint32_t uid = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
    snprintf(MQTT_CLIENT_ID, sizeof(MQTT_CLIENT_ID), "bg77-%08lX", (unsigned long)uid);
}

void Bg77Mqtt::loop()
{
    pumpModem(10);

    if (_mqttConnected)
    {
        _disconnectedSinceMs = 0;
        if (_lastMqttOkMs != 0 && (millis() - _lastMqttOkMs) > MQTT_ACTIVITY_WATCHDOG_MS)
        {
            markMqttLost("keepalive stale");
        }
        return;
    }

    if (_disconnectedSinceMs == 0)
    {
        _disconnectedSinceMs = millis();
    }

    if ((millis() - _disconnectedSinceMs) >= MAX_DISCONNECTED_MS)
    {
        logError("LTE inaccessible depuis 30min, reboot");
        delay(100);
        NVIC_SystemReset();
    }

    uint32_t now = millis();
    if ((int32_t)(now - _nextReconnectAtMs) < 0)
    {
        return;
    }

    logInfo("Tentative init/reconnexion BG77 en arriere-plan");

    bool ok = init();
    if (ok)
    {
        _nextReconnectAtMs = 0;
    }
    else
    {
        if (_mqttReconnectFailures < 10)
            _mqttReconnectFailures++;
        _nextReconnectAtMs = millis() + reconnectBackoffMs();
    }
}

bool Bg77Mqtt::isNetworkRegistered() const { return _networkRegistered; }
bool Bg77Mqtt::isPacketAttached() const { return _packetAttached; }
bool Bg77Mqtt::isMqttConnected() const { return _mqttConnected; }

void Bg77Mqtt::setKeepAlive(uint16_t keepAliveSec)
{
    if (keepAliveSec > 0)
    {
        _mqttKeepAliveSec = keepAliveSec;
    }
}

bool Bg77Mqtt::hasNewMqttMessage() const { return _mqttQueueCount > 0; }
String Bg77Mqtt::getLastMqttMessage() const { return (_mqttQueueCount > 0) ? _mqttQueue[_mqttQueueHead] : ""; }

void Bg77Mqtt::clearLastMqttMessage()
{
    if (_mqttQueueCount > 0)
    {
        _mqttQueue[_mqttQueueHead] = "";
        _mqttQueueHead = (_mqttQueueHead + 1) % MQTT_QUEUE_SIZE;
        _mqttQueueCount--;
    }
}

void Bg77Mqtt::logInfo(const String &msg)
{
    _pc.print("[INFO] ");
    _pc.println(msg);
}

void Bg77Mqtt::logWarn(const String &msg)
{
    _pc.print("[WARN] ");
    _pc.println(msg);
}

void Bg77Mqtt::logError(const String &msg)
{
    _pc.print("[ERR ] ");
    _pc.println(msg);
}

void Bg77Mqtt::bg77PowerPulse()
{
    pinMode(_pwrKeyPin, OUTPUT);

    digitalWrite(_pwrKeyPin, BG77_PWRKEY_IDLE_LEVEL);
    delay(50);

    digitalWrite(_pwrKeyPin, BG77_PWRKEY_ACTIVE_LEVEL);
    delay(BG77_PWRKEY_PULSE_MS);

    digitalWrite(_pwrKeyPin, BG77_PWRKEY_IDLE_LEVEL);
    delay(1500);
}

void Bg77Mqtt::clearModemInput()
{
    while (_modem.available())
    {
        _modem.read();
    }
}

void Bg77Mqtt::printStateSnapshot(const String &origin)
{
    _pc.print("[STATE] ");
    _pc.print(origin);
    _pc.print(" | reg=");
    _pc.print(_networkRegistered ? "1" : "0");
    _pc.print(" att=");
    _pc.print(_packetAttached ? "1" : "0");
    _pc.print(" sock=");
    _pc.print(_mqttSocketOpen ? "1" : "0");
    _pc.print(" mqtt=");
    _pc.print(_mqttConnected ? "1" : "0");
    _pc.print(" ka=");
    _pc.print(_mqttKeepAliveSec);
    _pc.println("s");
}

void Bg77Mqtt::handleModemLine(const String &line)
{
    if (line.length() == 0)
    {
        return;
    }

    _pc.print("[MODEM] ");
    _pc.println(line);

    if (line.indexOf("+CEREG:") >= 0)
    {
        int comma = line.indexOf(',');
        bool registered = false;

        if (comma >= 0)
        {
            char stat = line.charAt(comma + 1);
            if (stat == '1' || stat == '5')
            {
                char after = (comma + 2 < (int)line.length()) ? line.charAt(comma + 2) : '\n';
                if (after == ',' || after == '\r' || after == '\n' || after == '\0' || after == ' ')
                {
                    registered = true;
                }
            }
        }

        _networkRegistered = registered;
        printStateSnapshot("CEREG");
    }

    if (line.indexOf("+CGATT: 1") >= 0)
    {
        _packetAttached = true;
        printStateSnapshot("CGATT");
    }
    else if (line.indexOf("+CGATT: 0") >= 0)
    {
        _packetAttached = false;
        printStateSnapshot("CGATT");
    }

    if (line.indexOf("+QIURC: \"pdpdeact\",1") >= 0)
    {
        _packetAttached = false;
        _mqttSocketOpen = false;
        _mqttConnected = false;
        logWarn("PDP deactivated");
        printStateSnapshot("PDPDEACT");
    }

    if (line.indexOf("+QMTOPEN: 0,0") >= 0)
    {
        _mqttSocketOpen = true;
        printStateSnapshot("QMTOPEN OK");
    }
    else if (line.startsWith("+QMTOPEN: 0,"))
    {
        _mqttSocketOpen = false;
        _mqttConnected = false;
        printStateSnapshot("QMTOPEN FAIL");
    }

    if (line.indexOf("+QMTCONN: 0,0,0") >= 0)
    {
        _mqttConnected = true;
        _mqttReconnectFailures = 0;
        _lastMqttOkMs = millis();
        printStateSnapshot("QMTCONN OK");
    }
    else if (line.startsWith("+QMTCONN: 0,"))
    {
        _mqttConnected = false;
        printStateSnapshot("QMTCONN FAIL");
    }

    if (line.startsWith("+QMTSTAT:"))
    {
        _mqttConnected = false;
        printStateSnapshot("QMTSTAT");
    }

    if (line.startsWith("+QMTCLOSE:"))
    {
        _mqttSocketOpen = false;
        _mqttConnected = false;
        printStateSnapshot("QMTCLOSE");
    }

    if (line.startsWith("+QMTDISC:"))
    {
        _mqttConnected = false;
        printStateSnapshot("QMTDISC");
    }

    if (line.startsWith("+QMTRECV:"))
    {
        int firstComma = line.indexOf(',');
        int secondComma = (firstComma >= 0) ? line.indexOf(',', firstComma + 1) : -1;
        int topicOpen = (secondComma >= 0) ? line.indexOf('"', secondComma + 1) : -1;
        int topicClose = (topicOpen >= 0) ? line.indexOf('"', topicOpen + 1) : -1;
        int payloadOpen = (topicClose >= 0) ? line.indexOf('"', topicClose + 1) : -1;
        int payloadClose = line.lastIndexOf('"');

        if (payloadOpen >= 0 && payloadClose > payloadOpen)
        {
            if (_mqttQueueCount < MQTT_QUEUE_SIZE)
            {
                _mqttQueue[_mqttQueueTail] = line.substring(payloadOpen + 1, payloadClose);
                _mqttQueueTail = (_mqttQueueTail + 1) % MQTT_QUEUE_SIZE;
                _mqttQueueCount++;
            }
            else
            {
                logWarn("MQTT queue pleine, message perdu");
            }
        }
    }
}

void Bg77Mqtt::pumpModem(uint32_t durationMs)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < durationMs)
    {
        watchdogFeed();

        while (_modem.available())
        {
            char c = (char)_modem.read();

            if (c == '\r')
            {
                continue;
            }

            if (c == '\n')
            {
                _modemLine.trim();
                if (_modemLine.length() > 0)
                {
                    handleModemLine(_modemLine);
                }
                _modemLine = "";
            }
            else
            {
                _modemLine += c;
            }
        }

        delay(1);
    }
}

bool Bg77Mqtt::sendATWaitFor(const String &cmd,
                             const char *token1,
                             const char *token2,
                             uint32_t timeoutMs,
                             String *fullResponse)
{
    clearModemInput();

    _pc.print(">> ");
    _pc.println(cmd);

    _modem.println(cmd);

    String response = "";
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs)
    {
        watchdogFeed();

        while (_modem.available())
        {
            char c = (char)_modem.read();

            if (c == '\r')
            {
                continue;
            }

            if (c == '\n')
            {
                _modemLine.trim();

                if (_modemLine.length() > 0)
                {
                    response += _modemLine;
                    response += "\n";
                    handleModemLine(_modemLine);

                    if (token1 != nullptr && _modemLine.indexOf(token1) >= 0)
                    {
                        if (fullResponse != nullptr)
                        {
                            *fullResponse = response;
                        }
                        return true;
                    }

                    if (token2 != nullptr && _modemLine.indexOf(token2) >= 0)
                    {
                        if (fullResponse != nullptr)
                        {
                            *fullResponse = response;
                        }
                        return false;
                    }
                }

                _modemLine = "";
            }
            else
            {
                _modemLine += c;
            }
        }

        delay(1);
    }

    if (fullResponse != nullptr)
    {
        *fullResponse = response;
    }

    return false;
}

bool Bg77Mqtt::sendATOK(const String &cmd, uint32_t timeoutMs)
{
    return sendATWaitFor(cmd, "OK", "ERROR", timeoutMs, nullptr);
}

bool Bg77Mqtt::isModemAlive()
{
    return sendATOK("AT", 3000);
}

bool Bg77Mqtt::waitModemReady(uint32_t timeoutMs)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs)
    {
        watchdogFeed();

        if (isModemAlive())
        {
            logInfo("BG77 repond a AT");

            sendATOK("ATE0", 3000);
            sendATOK("AT+CMEE=2", 3000);

            return true;
        }

        delay(500);
    }

    return false;
}

bool Bg77Mqtt::prepareModemAfterPowerOn()
{
    if (!waitModemReady(30000UL))
    {
        return false;
    }

    bool simReady = false;
    uint32_t simStart = millis();

    while ((uint32_t)(millis() - simStart) < 30000UL)
    {
        watchdogFeed();

        if (sendATWaitFor("AT+CPIN?", "+CPIN: READY", "ERROR", 5000, nullptr))
        {
            simReady = true;
            break;
        }

        logWarn("SIM pas encore READY");
        delay(1000);
    }

    if (!simReady)
    {
        logError("SIM non READY apres attente");
        return false;
    }

    sendATOK("AT+CPSMS=0", 5000);
    sendATOK("AT+CEDRXS=0,5", 5000);
    sendATOK("AT+QCFG=\"psm/urc\",1", 5000);

    return true;
}

bool Bg77Mqtt::bg77PowerCycle()
{
    logWarn("Power cycle BG77: OFF puis ON");

    _networkRegistered = false;
    _packetAttached = false;
    _mqttSocketOpen = false;
    _mqttConnected = false;

    if (isModemAlive())
    {
        logInfo("BG77 vivant, extinction propre AT+QPOWD=1");

        sendATWaitFor("AT+QPOWD=1", "OK", "ERROR", 5000, nullptr);

        uint32_t offStart = millis();
        while ((uint32_t)(millis() - offStart) < 15000UL)
        {
            watchdogFeed();

            if (!isModemAlive())
            {
                logInfo("BG77 eteint proprement");
                break;
            }

            delay(1000);
        }
    }

    if (isModemAlive())
    {
        logWarn("BG77 encore vivant, pulse PWRKEY pour extinction");

        bg77PowerPulse();

        uint32_t offStart = millis();
        while ((uint32_t)(millis() - offStart) < 15000UL)
        {
            watchdogFeed();

            if (!isModemAlive())
            {
                logInfo("BG77 eteint apres pulse PWRKEY");
                break;
            }

            delay(1000);
        }
    }

    delay(3000);

    logInfo("Pulse PWRKEY pour rallumer BG77");
    bg77PowerPulse();

    if (!prepareModemAfterPowerOn())
    {
        logError("BG77/SIM pas pret apres rallumage");
        return false;
    }

    logInfo("BG77 pret apres power cycle");
    return true;
}

bool Bg77Mqtt::updateRegistration()
{
    String response;
    if (!sendATWaitFor("AT+CEREG?", "+CEREG:", nullptr, 5000, &response))
    {
        return false;
    }

    return _networkRegistered;
}

bool Bg77Mqtt::waitForRegistration(uint32_t timeoutMs)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs)
    {
        watchdogFeed();

        if (updateRegistration())
        {
            return true;
        }

        delay(3000);
    }

    return false;
}

bool Bg77Mqtt::updatePacketAttach()
{
    String response;
    if (!sendATWaitFor("AT+CGATT?", "+CGATT:", nullptr, 5000, &response))
    {
        return false;
    }

    return true;
}

bool Bg77Mqtt::hasIPAddress()
{
    String response;
    if (!sendATWaitFor("AT+CGPADDR=1", "+CGPADDR:", nullptr, 5000, &response))
    {
        return false;
    }

    int comma = response.indexOf(',');
    if (comma < 0)
    {
        return false;
    }

    String ip = response.substring(comma + 1);
    ip.trim();

    return ip.length() >= 7;
}

static int extractNthFieldBg(const String &s, int n)
{
    int start = 0;
    for (int i = 0; i < n; i++)
    {
        int comma = s.indexOf(',', start);
        if (comma < 0)
            return -999;
        start = comma + 1;
    }
    int end = s.indexOf(',', start);
    String val = (end >= 0) ? s.substring(start, end) : s.substring(start);
    val.trim();
    if (val.length() == 0)
        return -999;
    return (int)val.toInt();
}

static String extractQcsqLineBg(const String &resp)
{
    int pos = resp.indexOf("+QCSQ:");
    if (pos < 0)
    {
        return "";
    }

    int endN = resp.indexOf('\n', pos);
    int endR = resp.indexOf('\r', pos);
    int end = -1;

    if (endN >= 0 && endR >= 0)
    {
        end = min(endN, endR);
    }
    else if (endN >= 0)
    {
        end = endN;
    }
    else if (endR >= 0)
    {
        end = endR;
    }

    String line = (end >= 0) ? resp.substring(pos, end) : resp.substring(pos);
    line.trim();
    return line;
}

static String qcsqTechBg(const String &resp)
{
    String line = extractQcsqLineBg(resp);
    if (line.length() == 0)
    {
        return "UNKNOWN";
    }

    int openQuote = line.indexOf('"');
    int closeQuote = (openQuote >= 0) ? line.indexOf('"', openQuote + 1) : -1;
    if (openQuote < 0 || closeQuote <= openQuote)
    {
        return "UNKNOWN";
    }

    String tech = line.substring(openQuote + 1, closeQuote);
    tech.replace("LTE NB-IoT", "NB-IoT");
    tech.replace("NBIoT", "NB-IoT");
    tech.replace("LTE Cat-M1", "Cat-M1");
    tech.replace("eMTC", "Cat-M1");
    tech.trim();
    return tech;
}

static String qcsqValuesBg(const String &resp)
{
    String line = extractQcsqLineBg(resp);
    if (line.length() == 0)
    {
        return "";
    }

    int closeQuote = line.lastIndexOf('"');
    if (closeQuote < 0)
    {
        return "";
    }

    String vals = line.substring(closeQuote + 1);
    vals.trim();

    if (vals.startsWith(","))
    {
        vals = vals.substring(1);
    }

    vals.replace("\r", "");
    vals.replace("\n", "");
    vals.replace("\"", "");
    vals.trim();
    return vals;
}

int Bg77Mqtt::extractRsrpFromQcsq(const String &qcsqResp)
{
    String vals = qcsqValuesBg(qcsqResp);
    if (vals.length() == 0)
    {
        return -999;
    }

    // +QCSQ: "...",<RSSI>,<RSRP>,<SINR>,<RSRQ>
    // RSSI is field 0, RSRP is field 1.
    return extractNthFieldBg(vals, 1);
}

bool Bg77Mqtt::tryNetworkProfile(const char *name,
                                 const char *plmn,
                                 int act,
                                 int iotopmode,
                                 int *outRsrp)
{
    logInfo(String("Essai profil LTE: ") + name);

    _networkRegistered = false;
    _packetAttached = false;

    {
        String cmd = "AT+QCFG=\"iotopmode\",";
        cmd += String(iotopmode);
        cmd += ",1";

        if (!sendATOK(cmd, 5000))
        {
            logWarn(String("QCFG iotopmode refuse pour ") + name);
            return false;
        }
    }

    {
        String cmd = "AT+COPS=1,2,\"";
        cmd += plmn;
        cmd += "\",";
        cmd += String(act);

        if (!sendATOK(cmd, 30000))
        {
            logWarn(String("COPS refuse pour ") + name);
            return false;
        }
    }

    if (!waitForRegistration(45000))
    {
        logWarn(String("Pas d'enregistrement reseau pour ") + name);
        return false;
    }

    _lastProfileName = String(name);
    _lteStatus = String(name);
    _lteStatus.replace(" NB-IoT", ";NB-IoT");
    _lteStatus.replace(" Cat-M1", ";Cat-M1");

    logInfo(String("Profil LTE OK: ") + name);

    String selectedQcsqResp = "";
    int finalRsrp = -999;
    uint8_t sampleCount = (outRsrp == nullptr) ? 1 : LTE_SIGNAL_SAMPLE_COUNT;

    for (uint8_t i = 0; i < sampleCount; i++)
    {
        watchdogFeed();

        if (i > 0)
        {
            delay(LTE_SIGNAL_SAMPLE_GAP_MS);
        }

        delay(100);

        while (_modem.available())
        {
            _modem.read();
        }

        String qcsqResp;

        if (sendATWaitFor("AT+QCSQ", "+QCSQ:", nullptr, 5000, &qcsqResp))
        {
            int rsrp = extractRsrpFromQcsq(qcsqResp);

            logInfo(
                String("QCSQ mesure ") +
                String(i + 1) +
                "/" +
                String(sampleCount) +
                " pour " +
                String(name) +
                ": RSRP=" +
                String(rsrp));

            if (rsrp > -999)
            {
                /*
                 * Conservative choice:
                 * keep the worst RSRP observed during the sampling window.
                 *
                 * This avoids selecting a profile only because of a short
                 * optimistic peak just after registration.
                 */
                if (finalRsrp == -999 || rsrp < finalRsrp)
                {
                    finalRsrp = rsrp;
                    selectedQcsqResp = qcsqResp;
                }
            }
        }
        else
        {
            logWarn(
                String("QCSQ mesure ") +
                String(i + 1) +
                "/" +
                String(LTE_SIGNAL_SAMPLE_COUNT) +
                " indisponible pour " +
                String(name));
        }
    }

    if (finalRsrp > -999)
    {
        String vals = qcsqValuesBg(selectedQcsqResp);

        if (vals.length() > 0)
        {
            vals.replace(",", ";");
            vals.replace("\"", "");
            vals.replace("\r", "");
            vals.replace("\n", "");
            vals.trim();

            _lteStatus += ";";
            _lteStatus += vals;
        }

        if (outRsrp != nullptr)
        {
            *outRsrp = finalRsrp;
        }

        logInfo(
            String("QCSQ final pour ") +
            String(name) +
            ": worst_rsrp=" +
            String(finalRsrp));
    }
    else
    {
        if (outRsrp != nullptr)
        {
            *outRsrp = -999;
        }

        logWarn(String("QCSQ indisponible pour ") + String(name));
    }

    delay(100);

    while (_modem.available())
    {
        _modem.read();
    }

    updateProviderFromCops();

    return true;
}

bool Bg77Mqtt::selectBestNbIot(bool *outPreferBouygues)
{
    logInfo("[SCAN NB-IoT] Debut scan qualite signal...");

    int rsrpBouygues = -999;
    bool bouyguesOk = tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 1, &rsrpBouygues);
    if (bouyguesOk)
        logInfo(String("[SCAN NB-IoT] Bouygues RSRP=") + String(rsrpBouygues) + " dBm");
    else
        logWarn("[SCAN NB-IoT] Bouygues NB-IoT indisponible");

    // Soft deregister before scanning Orange
    sendATOK("AT+COPS=2", 15000);
    delay(2000);

    int rsrpOrange = -999;
    bool orangeOk = tryNetworkProfile("Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 1, &rsrpOrange);
    if (orangeOk)
        logInfo(String("[SCAN NB-IoT] Orange RSRP=") + String(rsrpOrange) + " dBm");
    else
        logWarn("[SCAN NB-IoT] Orange NB-IoT indisponible");

    // Determine preferred operator (used also to order Cat-M1 fallback if NB-IoT fails)
    bool preferBouygues;
    if (!bouyguesOk && !orangeOk)
        preferBouygues = true; // no signal info — default to Bouygues
    else
        preferBouygues = bouyguesOk && (!orangeOk || rsrpBouygues >= rsrpOrange);

    if (outPreferBouygues != nullptr)
        *outPreferBouygues = preferBouygues;

    if (!bouyguesOk && !orangeOk)
    {
        logWarn("[SCAN NB-IoT] Aucun operateur NB-IoT disponible");
        return false;
    }

    if (preferBouygues)
    {
        logInfo(String("[SCAN NB-IoT] Meilleur: Bouygues RSRP=") + String(rsrpBouygues) + " dBm vs Orange=" + String(rsrpOrange) + " dBm");
        // Orange may be currently registered — deregister before switching
        if (orangeOk)
        {
            sendATOK("AT+COPS=2", 15000);
            delay(2000);
        }
        return tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 1);
    }

    // Orange is best and is currently the active registration
    logInfo(String("[SCAN NB-IoT] Meilleur: Orange RSRP=") + String(rsrpOrange) + " dBm vs Bouygues=" + String(rsrpBouygues) + " dBm");
    return true;
}

bool Bg77Mqtt::selectBestLteProfile()
{
    logInfo("[SCAN LTE] Debut scan complet Bouygues/Orange NB-IoT/Cat-M1");

    LteProfileCandidate profiles[] = {
        {"Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 1, -999, false},
        {"Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 1, -999, false},
        {"Bouygues Cat-M1", PLMN_BOUYGUES, ACT_CATM1, 0, -999, false},
        {"Orange Cat-M1", PLMN_ORANGE, ACT_CATM1, 0, -999, false},
    };

    const uint8_t profileCount = sizeof(profiles) / sizeof(profiles[0]);

    int bestIndex = -1;
    int bestScore = -9999;

    for (uint8_t i = 0; i < profileCount; i++)
    {
        watchdogFeed();

        if (i > 0)
        {
            sendATOK("AT+COPS=2", 15000);
            delay(3000);
        }

        profiles[i].ok = tryNetworkProfile(
            profiles[i].name,
            profiles[i].plmn,
            profiles[i].act,
            profiles[i].iotopmode,
            &profiles[i].rsrp);

        if (!profiles[i].ok)
        {
            logWarn(String("[SCAN LTE] Retry profil ") + profiles[i].name);

            sendATOK("AT+COPS=2", 15000);
            delay(5000);

            profiles[i].ok = tryNetworkProfile(
                profiles[i].name,
                profiles[i].plmn,
                profiles[i].act,
                profiles[i].iotopmode,
                &profiles[i].rsrp);
        }

        if (profiles[i].ok)
        {
            int score = profiles[i].rsrp;

            /*
             * Prefer NB-IoT when the difference is small.
             *
             * Example:
             * B_NBIOT = -92, B_CATM1 = -91
             *
             * Without bonus, Cat-M1 wins.
             * With +5 dB bonus:
             * NB-IoT score = -87
             * Cat-M1 score = -91
             * NB-IoT wins.
             */
            if (profiles[i].act == ACT_NBIOT)
            {
                score += NBIOT_PREFERENCE_DB;
            }

            logInfo(String("[SCAN LTE] ") + profiles[i].name +
                    " OK RSRP=" + String(profiles[i].rsrp) +
                    " dBm score=" + String(score));

            if (bestIndex < 0 || score > bestScore)
            {
                bestIndex = i;
                bestScore = score;
            }
        }
        else
        {
            logWarn(String("[SCAN LTE] ") + profiles[i].name + " indisponible");
        }
    }

    _lastScanSummary = "";

    for (uint8_t i = 0; i < profileCount; i++)
    {
        if (i > 0)
        {
            _lastScanSummary += ";";
        }

        if (strcmp(profiles[i].name, "Bouygues NB-IoT") == 0)
        {
            _lastScanSummary += "B_NBIOT=";
        }
        else if (strcmp(profiles[i].name, "Orange NB-IoT") == 0)
        {
            _lastScanSummary += "O_NBIOT=";
        }
        else if (strcmp(profiles[i].name, "Bouygues Cat-M1") == 0)
        {
            _lastScanSummary += "B_CATM1=";
        }
        else if (strcmp(profiles[i].name, "Orange Cat-M1") == 0)
        {
            _lastScanSummary += "O_CATM1=";
        }
        else
        {
            _lastScanSummary += "UNKNOWN=";
        }

        if (profiles[i].ok)
        {
            _lastScanSummary += String(profiles[i].rsrp);
        }
        else
        {
            _lastScanSummary += "-999";
        }
    }

    if (bestIndex < 0)
    {
        logWarn("[SCAN LTE] Aucun profil LTE disponible");
        return false;
    }

    logInfo(String("[SCAN LTE] Meilleur profil: ") + profiles[bestIndex].name +
            " RSRP=" + String(profiles[bestIndex].rsrp) +
            " dBm score=" + String(bestScore));

    sendATOK("AT+COPS=2", 15000);
    delay(3000);

    bool selected = tryNetworkProfile(
        profiles[bestIndex].name,
        profiles[bestIndex].plmn,
        profiles[bestIndex].act,
        profiles[bestIndex].iotopmode);

    if (!selected)
    {
        logWarn(String("[SCAN LTE] Echec selection meilleur profil: ") + profiles[bestIndex].name);
        return false;
    }

    return true;
}

bool Bg77Mqtt::init()
{
    logInfo("Init complete LTE full scan + MQTT");

    if (!_bootPowerCycleDone)
    {
        logWarn("Power cycle BG77 au boot");
        bg77PowerCycle();
        _bootPowerCycleDone = true;
    }

    if (!prepareModemAfterPowerOn())
    {
        logError("BG77 non pret");
        return false;
    }

    updateRegistration();

    bool registered = _networkRegistered;

    if (!_bootProviderScanDone || !registered)
    {
        logInfo("Stabilisation radio avant scan LTE complet");

        sendATOK("AT+QCFG=\"iotopmode\",1,1", 5000);

        uint32_t stableStart = millis();
        while ((uint32_t)(millis() - stableStart) < 30000UL)
        {
            watchdogFeed();
            delay(1000);
        }

        registered = selectBestLteProfile();
        _bootProviderScanDone = true;

        if (!registered)
        {
            logWarn("All LTE profiles failed, final BG77 power cycle");
            bg77PowerCycle();
            logError("Aucun profil LTE disponible");
            return false;
        }
    }
    else
    {
        logInfo("Deja enregistre, reconnect direct sans nouveau scan LTE complet");
    }

    if (registered)
    {
        logInfo("LTE enregistre, lecture statut radio");

        String qcsqResp;
        bool haveQcsq = sendATWaitFor("AT+QCSQ", "+QCSQ:", nullptr, 5000, &qcsqResp);

        String base;
        if (_lastProfileName.length() > 0)
        {
            base = _lastProfileName;
            base.replace(" NB-IoT", ";NB-IoT");
            base.replace(" Cat-M1", ";Cat-M1");
        }
        else if (haveQcsq)
        {
            base = "?;" + qcsqTechBg(qcsqResp);
        }
        else
        {
            base = "?;UNKNOWN";
        }

        if (haveQcsq)
        {
            String vals = qcsqValuesBg(qcsqResp);
            if (vals.length() > 0)
            {
                vals.replace(",", ";");
                vals.replace("\r", "");
                vals.replace("\n", "");
                vals.replace("\"", "");
                vals.trim();

                _lteStatus = base + ";" + vals;
            }
            else
            {
                _lteStatus = base;
            }
        }
        else
        {
            _lteStatus = base;
        }

        updateProviderFromCops();
    }

    {
        String cmd = "AT+CGDCONT=1,\"IP\",\"";
        cmd += APN_NAME;
        cmd += "\"";
        if (!sendATOK(cmd, 5000))
        {
            logError("CGDCONT refuse");
            return false;
        }
    }

    if (!updatePacketAttach())
    {
        logError("CGATT? echec");
        return false;
    }

    if (!_packetAttached)
    {
        if (!sendATOK("AT+CGATT=1", 30000))
        {
            logError("CGATT=1 refuse");
            return false;
        }

        if (!updatePacketAttach())
        {
            logError("CGATT? apres attach echec");
            return false;
        }

        if (!_packetAttached)
        {
            logError("Packet attach absent");
            return false;
        }
    }

    if (!hasIPAddress())
    {
        logError("Pas d'adresse IP");
        return false;
    }

    if (!setMQTTKeepAliveInternal(_mqttKeepAliveSec))
    {
        return false;
    }

    return mqttReconnect();
}

bool Bg77Mqtt::ensureConnection()
{
    if (_mqttConnected)
    {
        return true;
    }

    return init();
}

bool Bg77Mqtt::ensureNetwork()
{
    if (!_networkRegistered)
    {
        if (!updateRegistration())
        {
            return false;
        }
    }

    if (!_packetAttached)
    {
        if (!updatePacketAttach())
        {
            return false;
        }
    }

    if (!_packetAttached)
    {
        if (!sendATOK("AT+CGATT=1", 30000))
        {
            return false;
        }

        if (!updatePacketAttach())
        {
            return false;
        }
    }

    return _networkRegistered && _packetAttached && hasIPAddress();
}

bool Bg77Mqtt::ensureMqtt()
{
    if (_mqttConnected)
    {
        return true;
    }

    if (!ensureNetwork())
    {
        return false;
    }

    return mqttReconnect();
}

bool Bg77Mqtt::setMQTTKeepAliveInternal(uint16_t keepAliveSec)
{
    String cmd = "AT+QMTCFG=\"keepalive\",0,";
    cmd += String(keepAliveSec);

    if (!sendATOK(cmd, 5000))
    {
        return false;
    }

    _mqttKeepAliveSec = keepAliveSec;
    return true;
}

void Bg77Mqtt::mqttDisconnectClose()
{
    sendATOK("AT+QMTDISC=0", 5000);
    sendATOK("AT+QMTCLOSE=0", 5000);

    _mqttSocketOpen = false;
    _mqttConnected = false;
    printStateSnapshot("DISCONNECT");
}

bool Bg77Mqtt::mqttSubscribe()
{
    String cmd = "AT+QMTSUB=0,1,\"";
    cmd += MQTT_TOPIC_SUB;
    cmd += "\",0";

    return sendATOK(cmd, 10000);
}

bool Bg77Mqtt::mqttReconnect()
{
    logInfo("Tentative reconnexion MQTT");

    mqttDisconnectClose();

    if (!setMQTTKeepAliveInternal(_mqttKeepAliveSec))
    {
        logError("Keepalive MQTT refuse");
        return false;
    }

    {
        String cmd = "AT+QMTOPEN=0,\"";
        cmd += BROKER_HOST;
        cmd += "\",";
        cmd += String(BROKER_PORT);
        String qmtopenResp;

        if (!sendATWaitFor(cmd, "+QMTOPEN:", "ERROR", 60000, &qmtopenResp))
        {
            logError("QMTOPEN echec: no response");
            _mqttSocketOpen = false;
            _mqttConnected = false;
            _mqttReconnectFailures++;
            return false;
        }

        if (qmtopenResp.indexOf("+QMTOPEN: 0,0") < 0)
        {
            logError("QMTOPEN echec: " + qmtopenResp);
            _mqttSocketOpen = false;
            _mqttConnected = false;
            _mqttReconnectFailures++;
            return false;
        }
    }

    {
        String cmd = "AT+QMTCONN=0,\"";
        cmd += MQTT_CLIENT_ID;
        cmd += "\",\"";
        cmd += MQTT_USER;
        cmd += "\",\"";
        cmd += MQTT_PASSWORD;
        cmd += "\"";

        if (!sendATWaitFor(cmd, "+QMTCONN: 0,0,0", "ERROR", 60000, nullptr))
        {
            logError("QMTCONN echec");
            _mqttConnected = false;
            _mqttReconnectFailures++;
            return false;
        }
    }

    if (!mqttSubscribe())
    {
        logError("QMTSUB echec");
        _mqttReconnectFailures++;
        return false;
    }

    _mqttConnected = true;
    _mqttSocketOpen = true;
    _mqttReconnectFailures = 0;
    _lastMqttOkMs = millis();
    _justConnected = true;

    logInfo("MQTT retabli");
    printStateSnapshot("MQTT RECONNECTED");

    return true;
}

bool Bg77Mqtt::publishToTopic(const String &topic, const String &payload)
{
    if (!_mqttConnected)
    {
        return false;
    }

    String cmd = "AT+QMTPUBEX=0,0,0,0,\"";
    cmd += topic;
    cmd += "\",\"";
    cmd += payload;
    cmd += "\"";

    if (!sendATWaitFor(cmd, "+QMTPUB: 0,0,0", "ERROR", 15000, nullptr))
    {
        markMqttLost("publish failed");
        return false;
    }

    _lastMqttOkMs = millis();
    return true;
}

bool Bg77Mqtt::publish(const String &payload)
{
    return publishToTopic(MQTT_TOPIC_PUB, payload);
}

bool Bg77Mqtt::publishAck(const String &payload)
{
    return publishToTopic(MQTT_TOPIC_ACK, payload);
}

void Bg77Mqtt::markMqttLost(const String &reason)
{
    logWarn(String("MQTT perdu: ") + reason);

    _mqttConnected = false;
    _mqttSocketOpen = false;
    _nextReconnectAtMs = millis() + reconnectBackoffMs();

    printStateSnapshot("MQTT LOST");
}

uint32_t Bg77Mqtt::reconnectBackoffMs() const
{
    if (_mqttReconnectFailures == 0)
    {
        return MQTT_RETRY_DELAY_MS;
    }

    uint32_t backoff = MQTT_RETRY_DELAY_MS;

    for (uint8_t i = 0; i < _mqttReconnectFailures; i++)
    {
        backoff *= 2UL;
        if (backoff > 120000UL)
        {
            backoff = 120000UL;
            break;
        }
    }

    return backoff;
}

bool Bg77Mqtt::publish(const String &payload, const char *topic)
{
    return publishToTopic(String(topic), payload);
}

void Bg77Mqtt::handleConsoleLine(const String &line)
{
    String trimmed = line;
    trimmed.trim();

    String lower = trimmed;
    lower.toLowerCase();

    if (lower == "pwr")
    {
        bg77PowerPulse();
        return;
    }

    if (lower == "reinit")
    {
        init();
        return;
    }

    if (lower == "disc")
    {
        mqttDisconnectClose();
        return;
    }

    if (lower.startsWith("ka "))
    {
        uint16_t keepAlive = (uint16_t)lower.substring(3).toInt();
        if (keepAlive > 0)
        {
            setKeepAlive(keepAlive);
            setMQTTKeepAliveInternal(keepAlive);
        }
        return;
    }

    if (lower == "at")
    {
        sendATOK("AT", 3000);
        return;
    }

    if (lower.startsWith("at "))
    {
        String raw = trimmed.substring(3);
        raw.trim();
        sendATWaitFor(raw, "OK", "ERROR", 10000, nullptr);
        return;
    }

    if (lower.startsWith("at+"))
    {
        sendATWaitFor(trimmed, "OK", "ERROR", 10000, nullptr);
        return;
    }
}

bool Bg77Mqtt::consumeJustConnected()
{
    bool value = _justConnected;
    _justConnected = false;
    return value;
}

String Bg77Mqtt::getNetworkDateTime()
{
    String response;

    if (!sendATWaitFor("AT+QLTS=2", "+QLTS:", "ERROR", 10000, &response))
    {
        return "";
    }

    int firstQuote = response.indexOf('"');
    int secondQuote = response.indexOf('"', firstQuote + 1);

    if (firstQuote < 0 || secondQuote <= firstQuote)
    {
        return "";
    }

    return response.substring(firstQuote + 1, secondQuote);
}

String Bg77Mqtt::convertQltsToIso8601(const String &qlts) const
{
    // Expected example:
    // 2026/04/20,20:04:24+08,0
    if (qlts.length() < 22)
    {
        return "";
    }

    int slash1 = qlts.indexOf('/');
    int slash2 = qlts.indexOf('/', slash1 + 1);
    int comma = qlts.indexOf(',');
    int colon1 = qlts.indexOf(':', comma + 1);
    int colon2 = qlts.indexOf(':', colon1 + 1);

    if (slash1 < 0 || slash2 < 0 || comma < 0 || colon1 < 0 || colon2 < 0)
    {
        return "";
    }

    String year = qlts.substring(0, slash1);
    String month = qlts.substring(slash1 + 1, slash2);
    String day = qlts.substring(slash2 + 1, comma);

    String hour = qlts.substring(comma + 1, colon1);
    String minute = qlts.substring(colon1 + 1, colon2);
    String second = qlts.substring(colon2 + 1, colon2 + 3);

    int signPosPlus = qlts.indexOf('+', colon2 + 1);
    int signPosMinus = qlts.indexOf('-', colon2 + 1);
    int signPos = -1;

    if (signPosPlus >= 0)
    {
        signPos = signPosPlus;
    }
    else if (signPosMinus >= 0)
    {
        signPos = signPosMinus;
    }

    String tz = "+00:00";

    if (signPos >= 0 && signPos + 3 <= (int)qlts.length())
    {
        char sign = qlts.charAt(signPos);
        int quarterHours = qlts.substring(signPos + 1, signPos + 3).toInt();
        int totalMinutes = quarterHours * 15;

        int hours = totalMinutes / 60;
        int minutes = totalMinutes % 60;

        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%c%02d:%02d", sign, hours, minutes);
        tz = String(buffer);
    }

    String iso = "";
    iso += year;
    iso += "-";
    iso += month;
    iso += "-";
    iso += day;
    iso += "T";
    iso += hour;
    iso += ":";
    iso += minute;
    iso += ":";
    iso += second;
    iso += tz;

    return iso;
}

String Bg77Mqtt::getNetworkDateTimeIso8601()
{
    String raw = getNetworkDateTime();
    if (raw.length() == 0)
    {
        return "";
    }

    return convertQltsToIso8601(raw);
}

String Bg77Mqtt::getLteStatus() const
{
    return _lteStatus;
}

String Bg77Mqtt::readLteSignal()
{
    if (!_mqttConnected)
    {
        return "";
    }

    String qcsqResp;
    if (!sendATWaitFor("AT+QCSQ", "+QCSQ:", nullptr, 5000, &qcsqResp))
    {
        return "";
    }

    String vals = qcsqValuesBg(qcsqResp);
    if (vals.length() == 0)
    {
        return "";
    }

    int rsrp = extractNthFieldBg(vals, 1);
    int rsrq = extractNthFieldBg(vals, 3);

    if (rsrp == -999 && rsrq == -999)
    {
        return "";
    }

    return String(rsrp) + ";" + String(rsrq);
}

String Bg77Mqtt::providerFromCopsResponse(const String &resp) const
{
    if (resp.indexOf("20820") >= 0 ||
        resp.indexOf("Bouygues") >= 0 ||
        resp.indexOf("BYTEL") >= 0)
    {
        return "Bouygues";
    }

    if (resp.indexOf("20801") >= 0 ||
        resp.indexOf("Orange") >= 0)
    {
        return "Orange";
    }

    return "?";
}

void Bg77Mqtt::updateProviderFromCops()
{
    String copsResp;
    String provider = "?";

    if (sendATWaitFor("AT+COPS?", "+COPS:", nullptr, 5000, &copsResp))
    {
        provider = providerFromCopsResponse(copsResp);
    }

    if (_lteStatus.indexOf(";") >= 0)
    {
        int sep = _lteStatus.indexOf(";");
        _lteStatus = provider + _lteStatus.substring(sep);
    }
    else
    {
        _lteStatus = provider + ";UNKNOWN";
    }
}

String Bg77Mqtt::getLteScanSummary() const
{
    return _lastScanSummary;
}
