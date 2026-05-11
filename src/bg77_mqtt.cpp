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
      _lteStatus("UNKNOWN;UNKNOWN"), //@@@ for mqtt connected
      _mqttKeepAliveSec(300),
      _lastMqttOkMs(0),
      _nextReconnectAtMs(0),
      _disconnectedSinceMs(0),
      _mqttReconnectFailures(0),
      _bootPowerCycleDone(false),
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

    if (!sendATWaitFor("AT+CPIN?", "+CPIN: READY", nullptr, 10000, nullptr))
    {
        logWarn("SIM pas encore READY");
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
        logError("BG77 ne repond pas apres rallumage");
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

bool Bg77Mqtt::tryNetworkProfile(const char *name,
                                 const char *plmn,
                                 int act,
                                 int iotopmode)
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
    _lteStatus = String(name);
    _lteStatus.replace(" NB-IoT", ";NB-IoT");
    _lteStatus.replace(" Cat-M1", ";Cat-M1");

    logInfo(String("Profil LTE OK: ") + name);

    String qcsqResp;
    sendATWaitFor("AT+QCSQ", "+QCSQ:", nullptr, 5000, &qcsqResp);
    int closeQuote = qcsqResp.lastIndexOf('"');
    if (closeQuote >= 0)
    {
        String vals = qcsqResp.substring(closeQuote + 1);
        vals.trim();
        if (vals.startsWith(","))
            vals = vals.substring(1);
        vals.replace(",", ";");
        if (vals.length() > 0)
        {
            _lteStatus += ";";
            _lteStatus += vals;
        }
    }

    sendATWaitFor("AT+COPS?", "+COPS:", nullptr, 5000, nullptr);

    return true;
}

bool Bg77Mqtt::init()
{
    logInfo("Init complete LTE fallback + MQTT");

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

    // Chemin rapide : si déjà enregistré, sauter la sélection réseau
    updateRegistration();

    if (!_networkRegistered)
    {
        sendATWaitFor("ATI", "OK", "ERROR", 5000, nullptr);

        bool registered = false;

        registered = tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 1);

        if (!registered)
        {
            logWarn("Echec Bouygues NB-IoT -> power cycle BG77");
            bg77PowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 1);

        if (!registered)
        {
            logWarn("Echec Orange NB-IoT -> power cycle BG77");
            bg77PowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Bouygues Cat-M1", PLMN_BOUYGUES, ACT_CATM1, 0);

        if (!registered)
        {
            logWarn("Echec Bouygues Cat-M1 -> power cycle BG77");
            bg77PowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Orange Cat-M1", PLMN_ORANGE, ACT_CATM1, 0);

        if (!registered)
        {
            logWarn("Echec Orange Cat-M1 -> power cycle BG77");
            bg77PowerCycle();
            logError("Aucun profil LTE disponible");
            return false;
        }
    }
    else
    {
        logInfo("Deja enregistre, reconnect direct");

        String qcsqResp;
        sendATWaitFor("AT+QCSQ", "+QCSQ:", nullptr, 5000, &qcsqResp);
        int closeQuote = qcsqResp.lastIndexOf('"');
        if (closeQuote >= 0)
        {
            int s1 = _lteStatus.indexOf(';');
            int s2 = (s1 >= 0) ? _lteStatus.indexOf(';', s1 + 1) : -1;
            String base = (s2 >= 0) ? _lteStatus.substring(0, s2) : _lteStatus;
            String vals = qcsqResp.substring(closeQuote + 1);
            vals.trim();
            if (vals.startsWith(","))
                vals = vals.substring(1);
            vals.replace(",", ";");
            if (vals.length() > 0)
                _lteStatus = base + ";" + vals;
        }

        sendATWaitFor("AT+COPS?", "+COPS:", nullptr, 5000, nullptr);
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

        if (!sendATWaitFor(cmd, "+QMTOPEN: 0,0", "ERROR", 60000, nullptr))
        {
            logError("QMTOPEN echec");
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

//@@@@ for mqtt connection
String Bg77Mqtt::getLteStatus() const
{
    return _lteStatus;
}
