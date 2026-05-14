/**
 * @file sim7080_mqtt.cpp
 * @brief SIMCom SIM7080 LTE/MQTT driver implementation.
 *
 * This file implements SIM7080 modem control through AT commands, including
 * startup preparation, LTE network profile selection, MQTT configuration,
 * subscribe/publish operations, inbound MQTT message queuing, and reconnection
 * watchdog behavior.
 */
#include "sim7080_mqtt.h"
#include "credentials.h"
#include "watchdog_simple.h"

static const char *BROKER_HOST = CRED_BROKER_HOST;     ///< MQTT broker hostname.
static const uint16_t BROKER_PORT = CRED_BROKER_PORT;  ///< MQTT broker TCP port.
static const char *MQTT_USER = CRED_MQTT_USER;         ///< MQTT username.
static const char *MQTT_PASSWORD = CRED_MQTT_PASSWORD; ///< MQTT password.
static const char *APN_NAME = CRED_APN_NAME;           ///< APN used for LTE packet service.

static char MQTT_CLIENT_ID[32] = "sim7080-client-000"; ///< Runtime MQTT client identifier.

static const char *MQTT_TOPIC_SUB = "d/t/cmd"; ///< MQTT topic subscribed for commands.
static const char *MQTT_TOPIC_PUB = "d/t";     ///< Default MQTT publish topic.
static const char *MQTT_TOPIC_ACK = "d/t/ack"; ///< MQTT acknowledgment publish topic.

static const char *PLMN_BOUYGUES = "20820"; ///< Bouygues Telecom PLMN.
static const char *PLMN_ORANGE = "20801";   ///< Orange France PLMN.

static const int ACT_CATM1 = 8; ///< SIM7080 access technology code for LTE Cat-M1.
static const int ACT_NBIOT = 9; ///< SIM7080 access technology code for NB-IoT.

static const uint32_t MQTT_RETRY_DELAY_MS = 2000; ///< Delay between immediate MQTT retry attempts, in milliseconds.

static const uint8_t SIM_PWRKEY_IDLE_LEVEL = LOW;    ///< Idle GPIO level for SIM7080 PWRKEY.
static const uint8_t SIM_PWRKEY_ACTIVE_LEVEL = HIGH; ///< Active GPIO level for SIM7080 PWRKEY pulse.
/**
 * @brief SIM7080G PWRKEY pulse duration; the modem requires more than 1 s to power on.
 */
static const uint32_t SIM_PWRKEY_PULSE_MS = 1200; // SIM7080G: >1s to power on

static const int NBIOT_PREFERENCE_DB = 5;
static const uint8_t LTE_SIGNAL_SAMPLE_COUNT = 6;
static const uint32_t LTE_SIGNAL_SAMPLE_GAP_MS = 5000UL;

static const uint32_t LTE_DEREGISTER_DELAY_MS = 10000UL;

struct LteProfileCandidate
{
    const char *name;
    const char *plmn;
    int act;
    int cmnb;
    int rsrp;
    bool ok;
};

/**
 * @brief Constructs the SIM7080 driver and initializes cached state.
 *
 */
Sim7080Mqtt::Sim7080Mqtt(Print &pcSerial, Uart &modemSerial, uint32_t pwrKeyPin)
    : _pc(pcSerial),
      _modem(modemSerial),
      _pwrKeyPin(pwrKeyPin),
      _modemLine(""),
      _expectingSubData(false),
      _networkRegistered(false),
      _packetAttached(false),
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

void Sim7080Mqtt::begin()
{
    _modem.begin(115200);
    delay(250);

    uint32_t uid = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
    snprintf(MQTT_CLIENT_ID, sizeof(MQTT_CLIENT_ID), "sim7080-%08lX", (unsigned long)uid);
}

void Sim7080Mqtt::loop()
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

    logInfo("Tentative init/reconnexion SIM7080G en arriere-plan");

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

bool Sim7080Mqtt::isNetworkRegistered() const { return _networkRegistered; }
bool Sim7080Mqtt::isPacketAttached() const { return _packetAttached; }
bool Sim7080Mqtt::isMqttConnected() const { return _mqttConnected; }

void Sim7080Mqtt::setKeepAlive(uint16_t keepAliveSec)
{
    if (keepAliveSec > 0)
    {
        _mqttKeepAliveSec = keepAliveSec;
    }
}

bool Sim7080Mqtt::hasNewMqttMessage() const { return _mqttQueueCount > 0; }
String Sim7080Mqtt::getLastMqttMessage() const { return (_mqttQueueCount > 0) ? _mqttQueue[_mqttQueueHead] : ""; }

void Sim7080Mqtt::clearLastMqttMessage()
{
    if (_mqttQueueCount > 0)
    {
        _mqttQueue[_mqttQueueHead] = "";
        _mqttQueueHead = (_mqttQueueHead + 1) % MQTT_QUEUE_SIZE;
        _mqttQueueCount--;
    }
}

void Sim7080Mqtt::logInfo(const String &msg)
{
    _pc.print("[INFO] ");
    _pc.println(msg);
}

void Sim7080Mqtt::logWarn(const String &msg)
{
    _pc.print("[WARN] ");
    _pc.println(msg);
}

void Sim7080Mqtt::logError(const String &msg)
{
    _pc.print("[ERR ] ");
    _pc.println(msg);
}

void Sim7080Mqtt::modemPowerPulse()
{
    pinMode(_pwrKeyPin, OUTPUT);
    digitalWrite(_pwrKeyPin, SIM_PWRKEY_IDLE_LEVEL);
    delay(50);
    digitalWrite(_pwrKeyPin, SIM_PWRKEY_ACTIVE_LEVEL);
    delay(SIM_PWRKEY_PULSE_MS);
    digitalWrite(_pwrKeyPin, SIM_PWRKEY_IDLE_LEVEL);
    delay(1500);
}

void Sim7080Mqtt::bg77PowerPulse()
{
    modemPowerPulse();
}

void Sim7080Mqtt::clearModemInput()
{
    while (_modem.available())
    {
        _modem.read();
    }
}

void Sim7080Mqtt::printStateSnapshot(const String &origin)
{
    _pc.print("[STATE] ");
    _pc.print(origin);
    _pc.print(" | reg=");
    _pc.print(_networkRegistered ? "1" : "0");
    _pc.print(" att=");
    _pc.print(_packetAttached ? "1" : "0");
    _pc.print(" mqtt=");
    _pc.print(_mqttConnected ? "1" : "0");
    _pc.print(" ka=");
    _pc.print(_mqttKeepAliveSec);
    _pc.println("s");
}

void Sim7080Mqtt::handleModemLine(const String &line)
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

    // +SMSTATE: 0 = disconnected URC, +SMSTATE: 1 = connected URC
    if (line.startsWith("+SMSTATE:"))
    {
        int colonPos = line.indexOf(':');
        if (colonPos >= 0)
        {
            String val = line.substring(colonPos + 1);
            val.trim();
            if (val == "0")
            {
                _mqttConnected = false;
                printStateSnapshot("SMSTATE DISC");
            }
            else if (val == "1")
            {
                _mqttConnected = true;
                _lastMqttOkMs = millis();
                printStateSnapshot("SMSTATE CONN");
            }
        }
    }

    // +SMSUB: "topic","payload"  — single-line URC per SIM7080G app note §5.1
    if (line.startsWith("+SMSUB:"))
    {
        _lastMqttOkMs = millis();
        // Extract payload: last ,"<payload>" portion
        int splitPos = line.lastIndexOf(",\"");
        if (splitPos >= 0)
        {
            String payload = line.substring(splitPos + 2);
            if (payload.length() > 0 && payload.charAt(payload.length() - 1) == '"')
            {
                payload = payload.substring(0, payload.length() - 1);
            }
            if (_mqttQueueCount < MQTT_QUEUE_SIZE)
            {
                _mqttQueue[_mqttQueueTail] = payload;
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

void Sim7080Mqtt::pumpModem(uint32_t durationMs)
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

bool Sim7080Mqtt::sendATWaitFor(const String &cmd,
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
                        _modemLine = "";
                        return true;
                    }

                    if (token2 != nullptr && _modemLine.indexOf(token2) >= 0)
                    {
                        if (fullResponse != nullptr)
                        {
                            *fullResponse = response;
                        }
                        _modemLine = "";
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

bool Sim7080Mqtt::sendATOK(const String &cmd, uint32_t timeoutMs)
{
    return sendATWaitFor(cmd, "OK", "ERROR", timeoutMs, nullptr);
}

bool Sim7080Mqtt::isModemAlive()
{
    return sendATOK("AT", 3000);
}

bool Sim7080Mqtt::waitModemReady(uint32_t timeoutMs)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs)
    {
        watchdogFeed();

        if (isModemAlive())
        {
            logInfo("SIM7080G repond a AT");
            sendATOK("ATE0", 3000);
            sendATOK("AT+CMEE=2", 3000);
            return true;
        }

        delay(500);
    }

    return false;
}

bool Sim7080Mqtt::prepareModemAfterPowerOn()
{
    if (!waitModemReady(30000UL))
    {
        return false;
    }

    if (!sendATWaitFor("AT+CPIN?", "+CPIN: READY", nullptr, 10000, nullptr))
    {
        logWarn("SIM pas encore READY");
        return false;
    }

    sendATOK("AT+CPSMS=0", 5000);
    sendATOK("AT+CEDRXS=0,5", 5000);

    return true;
}

bool Sim7080Mqtt::modemPowerCycle()
{
    logWarn("Power cycle SIM7080G: OFF puis ON");

    _networkRegistered = false;
    _packetAttached = false;
    _mqttConnected = false;

    if (isModemAlive())
    {
        logInfo("SIM7080G vivant, extinction propre AT+CPOF");
        sendATWaitFor("AT+CPOF", "OK", "ERROR", 5000, nullptr);

        uint32_t offStart = millis();
        while ((uint32_t)(millis() - offStart) < 15000UL)
        {
            watchdogFeed();
            if (!isModemAlive())
            {
                logInfo("SIM7080G eteint proprement");
                break;
            }
            delay(1000);
        }
    }

    if (isModemAlive())
    {
        logWarn("SIM7080G encore vivant, pulse PWRKEY pour extinction");
        modemPowerPulse();

        uint32_t offStart = millis();
        while ((uint32_t)(millis() - offStart) < 15000UL)
        {
            watchdogFeed();
            if (!isModemAlive())
            {
                logInfo("SIM7080G eteint apres pulse PWRKEY");
                break;
            }
            delay(1000);
        }
    }

    delay(3000);

    logInfo("Pulse PWRKEY pour rallumer SIM7080G");
    modemPowerPulse();

    if (!prepareModemAfterPowerOn())
    {
        logError("SIM7080G ne repond pas apres rallumage");
        return false;
    }

    logInfo("SIM7080G pret apres power cycle");
    return true;
}

bool Sim7080Mqtt::updateRegistration()
{
    String response;
    if (!sendATWaitFor("AT+CEREG?", "+CEREG:", nullptr, 5000, &response))
    {
        return false;
    }
    return _networkRegistered;
}

bool Sim7080Mqtt::waitForRegistration(uint32_t timeoutMs)
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

bool Sim7080Mqtt::updatePacketAttach()
{
    String response;
    if (!sendATWaitFor("AT+CGATT?", "+CGATT:", nullptr, 5000, &response))
    {
        return false;
    }
    return true;
}

bool Sim7080Mqtt::hasIPAddress()
{
    // AT+CNACT? returns: +CNACT: 0,1,"x.x.x.x"
    String response;
    if (!sendATWaitFor("AT+CNACT?", "+CNACT:", nullptr, 5000, &response))
    {
        return false;
    }

    int q1 = response.indexOf('"');
    int q2 = (q1 >= 0) ? response.indexOf('"', q1 + 1) : -1;
    if (q1 < 0 || q2 <= q1)
    {
        return false;
    }

    String ip = response.substring(q1 + 1, q2);
    return ip.length() >= 7 && ip != "0.0.0.0";
}

static int extractNthField(const String &s, int n)
{
    int start = 0;
    for (int i = 0; i < n; i++)
    {
        int comma = s.indexOf(',', start);
        if (comma < 0)
        {
            return -999;
        }
        start = comma + 1;
    }

    int end = s.indexOf(',', start);
    String val = (end >= 0) ? s.substring(start, end) : s.substring(start);
    val.trim();

    if (val.length() == 0)
    {
        return -999;
    }

    return (int)val.toInt();
}

static String extractCpsiLine(const String &resp)
{
    int pos = resp.indexOf("+CPSI:");
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

static String cpsiPayload(const String &resp)
{
    String line = extractCpsiLine(resp);
    if (line.length() == 0)
    {
        return "";
    }

    int colon = line.indexOf(':');
    if (colon < 0)
    {
        return "";
    }

    String payload = line.substring(colon + 1);
    payload.trim();
    return payload;
}

static String cpsiTech(const String &resp)
{
    String payload = cpsiPayload(resp);
    if (payload.length() == 0)
    {
        return "UNKNOWN";
    }

    int comma = payload.indexOf(',');
    String tech = (comma >= 0) ? payload.substring(0, comma) : payload;
    tech.trim();
    tech.toUpperCase();

    if (tech.indexOf("NB") >= 0)
    {
        return "NB-IoT";
    }

    if (tech.indexOf("CAT-M") >= 0 || tech.indexOf("LTE-M") >= 0 || tech.indexOf("M1") >= 0)
    {
        return "Cat-M1";
    }

    return "UNKNOWN";
}

static String cpsiSignalPart(const String &resp)
{
    String payload = cpsiPayload(resp);
    if (payload.length() == 0)
    {
        return "";
    }

    // Skip first two fields: radio type and registration state.
    int c1 = payload.indexOf(',');
    int c2 = (c1 >= 0) ? payload.indexOf(',', c1 + 1) : -1;
    if (c2 < 0)
    {
        return "";
    }

    String sigPart = payload.substring(c2 + 1);
    sigPart.trim();
    return sigPart;
}

int Sim7080Mqtt::extractRsrpFromCpsi(const String &cpsiResp)
{
    String sigPart = cpsiSignalPart(cpsiResp);
    if (sigPart.length() == 0)
    {
        return -999;
    }

    // SIM7080 CPSI after type/status:
    // MCC-MNC,TAC,CID,Band,EARFCN,DLBW,ULBW,RSRQ,RSRP,...
    return extractNthField(sigPart, 8);
}

int Sim7080Mqtt::extractRsrqFromCpsi(const String &cpsiResp)
{
    String sigPart = cpsiSignalPart(cpsiResp);
    if (sigPart.length() == 0)
    {
        return -999;
    }

    return extractNthField(sigPart, 7);
}

String Sim7080Mqtt::buildLteStatusFromCpsi(const String &base, const String &cpsiResp) const
{
    int rsrp = const_cast<Sim7080Mqtt *>(this)->extractRsrpFromCpsi(cpsiResp);
    int rsrq = const_cast<Sim7080Mqtt *>(this)->extractRsrqFromCpsi(cpsiResp);

    // Keep the same logical shape as Bg77Mqtt:
    // provider;tech;<aux1>;<RSRP>;<aux2>;<RSRQ>
    // SIM7080 CPSI does not expose the exact same four QCSQ fields, so aux fields are 0.
    String out = base;
    out += ";0;";
    out += String(rsrp);
    out += ";0;";
    out += String(rsrq);
    return out;
}

bool Sim7080Mqtt::tryNetworkProfile(const char *name, const char *plmn, int act, int cmnb, int *outRsrp)
{
    logInfo(String("Essai profil LTE: ") + name);

    _networkRegistered = false;
    _packetAttached = false;

    {
        // AT+CMNB: 1=LTE-M, 2=NB-IoT, 3=both.
        String cmd = "AT+CMNB=";
        cmd += String(cmnb);

        if (!sendATOK(cmd, 5000))
        {
            logWarn(String("CMNB refuse pour ") + name);
            return false;
        }
        delay(10000);
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

    String selectedCpsiResp = "";
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
        clearModemInput();

        String cpsiResp;

        if (sendATWaitFor("AT+CPSI?", "+CPSI:", nullptr, 5000, &cpsiResp))
        {
            int rsrp = extractRsrpFromCpsi(cpsiResp);

            logInfo(
                String("CPSI mesure ") +
                String(i + 1) +
                "/" +
                String(sampleCount) +
                " pour " +
                String(name) +
                ": RSRP=" +
                String(rsrp));

            if (rsrp > -999)
            {
                // Conservative choice: keep the worst RSRP observed.
                if (finalRsrp == -999 || rsrp < finalRsrp)
                {
                    finalRsrp = rsrp;
                    selectedCpsiResp = cpsiResp;
                }
            }
        }
        else
        {
            logWarn(
                String("CPSI mesure ") +
                String(i + 1) +
                "/" +
                String(LTE_SIGNAL_SAMPLE_COUNT) +
                " indisponible pour " +
                String(name));
        }
    }

    if (finalRsrp > -999)
    {
        _lteStatus = buildLteStatusFromCpsi(_lteStatus, selectedCpsiResp);

        if (outRsrp != nullptr)
        {
            *outRsrp = finalRsrp;
        }

        logInfo(
            String("CPSI final pour ") +
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

        logWarn(String("CPSI indisponible pour ") + String(name));
    }

    delay(100);
    clearModemInput();

    updateProviderFromCops();

    return true;
}

bool Sim7080Mqtt::configureMqtt()
{
    {
        String cmd = "AT+SMCONF=\"URL\",\"";
        cmd += BROKER_HOST;
        cmd += "\",";
        cmd += String(BROKER_PORT);
        if (!sendATOK(cmd, 5000))
        {
            logError("SMCONF URL refuse");
            return false;
        }
    }

    {
        String cmd = "AT+SMCONF=\"CLIENTID\",\"";
        cmd += MQTT_CLIENT_ID;
        cmd += "\"";
        if (!sendATOK(cmd, 5000))
        {
            logError("SMCONF CLIENTID refuse");
            return false;
        }
    }

    {
        String cmd = "AT+SMCONF=\"USERNAME\",\"";
        cmd += MQTT_USER;
        cmd += "\"";
        if (!sendATOK(cmd, 5000))
        {
            logError("SMCONF USERNAME refuse");
            return false;
        }
    }

    {
        String cmd = "AT+SMCONF=\"PASSWORD\",\"";
        cmd += MQTT_PASSWORD;
        cmd += "\"";
        if (!sendATOK(cmd, 5000))
        {
            logError("SMCONF PASSWORD refuse");
            return false;
        }
    }

    {
        String cmd = "AT+SMCONF=\"KEEPTIME\",";
        cmd += String(_mqttKeepAliveSec);
        if (!sendATOK(cmd, 5000))
        {
            logError("SMCONF KEEPTIME refuse");
            return false;
        }
    }

    sendATOK("AT+SMCONF=\"CLEANSS\",1", 5000);
    sendATOK("AT+SMCONF=\"QOS\",0", 5000);

    return true;
}

bool Sim7080Mqtt::setMQTTKeepAliveInternal(uint16_t keepAliveSec)
{
    if (keepAliveSec > 0)
    {
        _mqttKeepAliveSec = keepAliveSec;
    }

    String cmd = "AT+SMCONF=\"KEEPTIME\",";
    cmd += String(_mqttKeepAliveSec);

    if (!sendATOK(cmd, 5000))
    {
        logError("SMCONF KEEPTIME refuse");
        return false;
    }

    return true;
}

void Sim7080Mqtt::mqttDisconnect()
{
    sendATOK("AT+SMDISC", 5000);
    _mqttConnected = false;
    printStateSnapshot("DISCONNECT");
}

void Sim7080Mqtt::mqttDisconnectClose()
{
    mqttDisconnect();
}

bool Sim7080Mqtt::mqttSubscribe()
{
    String cmd = "AT+SMSUB=\"";
    cmd += MQTT_TOPIC_SUB;
    cmd += "\",0";

    return sendATOK(cmd, 10000);
}

bool Sim7080Mqtt::mqttReconnect()
{
    logInfo("Tentative reconnexion MQTT SIM7080G");

    mqttDisconnect();

    if (!configureMqtt())
    {
        logError("Configuration MQTT refusee");
        return false;
    }

    if (!sendATWaitFor("AT+SMCONN", "OK", "ERROR", 60000, nullptr))
    {
        logError("SMCONN echec");
        _mqttConnected = false;
        _mqttReconnectFailures++;
        return false;
    }

    if (!mqttSubscribe())
    {
        logError("SMSUB echec");
        _mqttReconnectFailures++;
        return false;
    }

    _mqttConnected = true;
    _mqttReconnectFailures = 0;
    _lastMqttOkMs = millis();
    _justConnected = true;

    logInfo("MQTT retabli");
    printStateSnapshot("MQTT RECONNECTED");

    return true;
}

bool Sim7080Mqtt::publishToTopic(const String &topic, const String &payload)
{
    if (!_mqttConnected)
    {
        return false;
    }

    // AT+SMPUB uses a two-phase protocol: send command, wait for '>' prompt,
    // then send payload + Ctrl-Z and wait for OK.
    String cmd = "AT+SMPUB=\"";
    cmd += topic;
    cmd += "\",";
    cmd += String(payload.length());
    cmd += ",0,0";

    _pc.print(">> ");
    _pc.println(cmd);
    clearModemInput();
    _modem.println(cmd);

    // Phase 1: wait for '>' prompt (character-level, not line-based)
    uint32_t start = millis();
    bool gotPrompt = false;
    while ((uint32_t)(millis() - start) < 10000UL && !gotPrompt)
    {
        watchdogFeed();
        while (_modem.available())
        {
            char c = (char)_modem.read();
            if (c == '>')
            {
                gotPrompt = true;
                break;
            }
        }
        if (!gotPrompt)
            delay(1);
    }

    if (!gotPrompt)
    {
        logError("SMPUB: pas de prompt '>'");
        markMqttLost("SMPUB no prompt");
        return false;
    }

    // Phase 2: send exactly payload.length() bytes — SIM7080G is length-terminated, no Ctrl-Z
    _modem.print(payload);

    // Phase 3: wait for OK or ERROR
    start = millis();
    while ((uint32_t)(millis() - start) < 15000UL)
    {
        watchdogFeed();
        while (_modem.available())
        {
            char c = (char)_modem.read();
            if (c == '\r')
                continue;
            if (c == '\n')
            {
                _modemLine.trim();
                if (_modemLine.length() > 0)
                {
                    handleModemLine(_modemLine);
                    if (_modemLine.indexOf("OK") >= 0)
                    {
                        _modemLine = "";
                        _lastMqttOkMs = millis();
                        return true;
                    }
                    if (_modemLine.indexOf("ERROR") >= 0)
                    {
                        _modemLine = "";
                        markMqttLost("SMPUB publish failed");
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

    markMqttLost("SMPUB timeout");
    return false;
}

bool Sim7080Mqtt::publish(const String &payload)
{
    return publishToTopic(MQTT_TOPIC_PUB, payload);
}

bool Sim7080Mqtt::publishAck(const String &payload)
{
    return publishToTopic(MQTT_TOPIC_ACK, payload);
}

bool Sim7080Mqtt::publish(const String &payload, const char *topic)
{
    return publishToTopic(String(topic), payload);
}

void Sim7080Mqtt::markMqttLost(const String &reason)
{
    logWarn(String("MQTT perdu: ") + reason);
    _mqttConnected = false;
    _nextReconnectAtMs = millis() + reconnectBackoffMs();
    printStateSnapshot("MQTT LOST");
}

uint32_t Sim7080Mqtt::reconnectBackoffMs() const
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

bool Sim7080Mqtt::selectBestNbIot(bool *outPreferBouygues)
{
    logInfo("[SCAN NB-IoT] Debut scan qualite signal...");

    int rsrpBouygues = -999;
    bool bouyguesOk = tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 2, &rsrpBouygues);
    if (bouyguesOk)
        logInfo(String("[SCAN NB-IoT] Bouygues RSRP=") + String(rsrpBouygues) + " dBm");
    else
        logWarn("[SCAN NB-IoT] Bouygues NB-IoT indisponible");

    // Soft deregister before scanning Orange
    sendATOK("AT+COPS=2", 15000);
    delay(LTE_DEREGISTER_DELAY_MS);

    int rsrpOrange = -999;
    bool orangeOk = tryNetworkProfile("Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 2, &rsrpOrange);
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
            delay(LTE_DEREGISTER_DELAY_MS);
        }
        return tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 2);
    }

    // Orange is best and is currently the active registration
    logInfo(String("[SCAN NB-IoT] Meilleur: Orange RSRP=") + String(rsrpOrange) + " dBm vs Bouygues=" + String(rsrpBouygues) + " dBm");
    return true;
}

bool Sim7080Mqtt::selectBestLteProfile()
{
    logInfo("[SCAN LTE] Debut scan complet Bouygues/Orange NB-IoT/Cat-M1 SIM7080G");

    LteProfileCandidate profiles[] = {
        {"Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 2, -999, false},
        {"Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 2, -999, false},
        {"Bouygues Cat-M1", PLMN_BOUYGUES, ACT_CATM1, 1, -999, false},
        {"Orange Cat-M1", PLMN_ORANGE, ACT_CATM1, 1, -999, false},
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
            delay(LTE_DEREGISTER_DELAY_MS);
        }

        profiles[i].ok = tryNetworkProfile(
            profiles[i].name,
            profiles[i].plmn,
            profiles[i].act,
            profiles[i].cmnb,
            &profiles[i].rsrp);

        if (profiles[i].ok)
        {
            int score = profiles[i].rsrp;

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
    delay(LTE_DEREGISTER_DELAY_MS);

    bool selected = tryNetworkProfile(
        profiles[bestIndex].name,
        profiles[bestIndex].plmn,
        profiles[bestIndex].act,
        profiles[bestIndex].cmnb);

    if (!selected)
    {
        logWarn(String("[SCAN LTE] Echec selection meilleur profil: ") + profiles[bestIndex].name);
        return false;
    }

    return true;
}

bool Sim7080Mqtt::init()
{
    logInfo("Init complete LTE full scan + MQTT SIM7080G");

    if (!_bootPowerCycleDone)
    {
        logWarn("Power cycle SIM7080G au boot");
        modemPowerCycle();
        _bootPowerCycleDone = true;
    }

    if (!prepareModemAfterPowerOn())
    {
        logError("SIM7080G non pret");
        return false;
    }

    updateRegistration();

    bool registered = _networkRegistered;

    if (!_bootProviderScanDone || !registered)
    {
        logInfo("Stabilisation radio avant scan LTE complet SIM7080G");

        sendATOK("AT+CMNB=2", 5000);

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
            logWarn("All LTE profiles failed, final SIM7080G power cycle");
            modemPowerCycle();
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
        logInfo("LTE enregistre, lecture statut radio SIM7080G");

        String cpsiResp;
        bool haveCpsi = sendATWaitFor("AT+CPSI?", "+CPSI:", nullptr, 5000, &cpsiResp);

        String base;
        if (_lastProfileName.length() > 0)
        {
            base = _lastProfileName;
            base.replace(" NB-IoT", ";NB-IoT");
            base.replace(" Cat-M1", ";Cat-M1");
        }
        else if (haveCpsi)
        {
            base = "?;" + cpsiTech(cpsiResp);
        }
        else
        {
            base = "?;UNKNOWN";
        }

        if (haveCpsi)
        {
            _lteStatus = buildLteStatusFromCpsi(base, cpsiResp);
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

    {
        String cmd = "AT+CNCFG=0,1,\"";
        cmd += APN_NAME;
        cmd += "\"";
        sendATOK(cmd, 5000);
    }

    if (!sendATOK("AT+CNACT=0,1", 30000))
    {
        logWarn("CNACT=0,1 refuse, reset contexte PDP");
        sendATOK("AT+CNACT=0,0", 15000);
        delay(1000);
        if (!sendATOK("AT+CNACT=0,1", 30000))
        {
            logError("CNACT=0,1 echec");
            return false;
        }
    }

    pumpModem(2000);

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

bool Sim7080Mqtt::ensureConnection()
{
    if (_mqttConnected)
    {
        return true;
    }
    return init();
}

bool Sim7080Mqtt::ensureNetwork()
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

bool Sim7080Mqtt::ensureMqtt()
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

void Sim7080Mqtt::handleConsoleLine(const String &line)
{
    String trimmed = line;
    trimmed.trim();

    String lower = trimmed;
    lower.toLowerCase();

    if (lower == "pwr")
    {
        modemPowerPulse();
        return;
    }

    if (lower == "reinit")
    {
        init();
        return;
    }

    if (lower == "disc")
    {
        mqttDisconnect();
        return;
    }

    if (lower.startsWith("ka "))
    {
        uint16_t keepAlive = (uint16_t)lower.substring(3).toInt();
        if (keepAlive > 0)
        {
            setKeepAlive(keepAlive);
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

bool Sim7080Mqtt::consumeJustConnected()
{
    bool value = _justConnected;
    _justConnected = false;
    return value;
}

String Sim7080Mqtt::getLteStatus() const
{
    return _lteStatus;
}

String Sim7080Mqtt::readLteSignal()
{
    if (!_mqttConnected)
        return "";

    String cpsiResp;
    sendATWaitFor("AT+CPSI?", "+CPSI:", nullptr, 5000, &cpsiResp);

    int cpsiIdx = cpsiResp.indexOf("+CPSI:");
    if (cpsiIdx < 0)
        return "";

    String cpsiLine = cpsiResp.substring(cpsiIdx + 6);
    cpsiLine.trim();

    // Skip type ("LTE NB") and status ("Online")
    int c1 = cpsiLine.indexOf(',');
    int c2 = (c1 >= 0) ? cpsiLine.indexOf(',', c1 + 1) : -1;
    if (c2 < 0)
        return "";

    // Remaining: MCC-MNC,TAC,CID,Band,EARFCN,DLBW,ULBW,RSRQ,RSRP,...
    String sigPart = cpsiLine.substring(c2 + 1);
    int rsrq = extractNthField(sigPart, 7);
    int rsrp = extractNthField(sigPart, 8);

    if (rsrp == -999 && rsrq == -999)
        return "";

    return String(rsrp) + ";" + String(rsrq);
}

String Sim7080Mqtt::getNetworkDateTime()
{
    String response;

    if (!sendATWaitFor("AT+CCLK?", "+CCLK:", "ERROR", 10000, &response))
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

String Sim7080Mqtt::convertQltsToIso8601(const String &qlts) const
{
    // SIM7080 AT+CCLK? example: 26/05/14,12:34:56+08
    // BG77 AT+QLTS=2 example: 2026/05/14,12:34:56+08,0
    String raw = qlts;
    raw.trim();

    int slash1 = raw.indexOf('/');
    int slash2 = raw.indexOf('/', slash1 + 1);
    int comma = raw.indexOf(',');
    int colon1 = raw.indexOf(':', comma + 1);
    int colon2 = raw.indexOf(':', colon1 + 1);

    if (slash1 < 0 || slash2 < 0 || comma < 0 || colon1 < 0 || colon2 < 0)
    {
        return "";
    }

    String year = raw.substring(0, slash1);
    if (year.length() == 2)
    {
        year = "20" + year;
    }

    String month = raw.substring(slash1 + 1, slash2);
    String day = raw.substring(slash2 + 1, comma);
    String hour = raw.substring(comma + 1, colon1);
    String minute = raw.substring(colon1 + 1, colon2);
    String second = raw.substring(colon2 + 1, colon2 + 3);

    int signPosPlus = raw.indexOf('+', colon2 + 1);
    int signPosMinus = raw.indexOf('-', colon2 + 1);
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

    if (signPos >= 0 && signPos + 3 <= (int)raw.length())
    {
        char sign = raw.charAt(signPos);
        int quarterHours = raw.substring(signPos + 1, signPos + 3).toInt();
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

String Sim7080Mqtt::getNetworkDateTimeIso8601()
{
    String raw = getNetworkDateTime();
    if (raw.length() == 0)
    {
        return "";
    }

    return convertQltsToIso8601(raw);
}

String Sim7080Mqtt::providerFromCopsResponse(const String &resp) const
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

void Sim7080Mqtt::updateProviderFromCops()
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

String Sim7080Mqtt::getLteScanSummary() const
{
    return _lastScanSummary;
}
