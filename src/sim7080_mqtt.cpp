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

bool Sim7080Mqtt::tryNetworkProfile(const char *name, const char *plmn, int act, int cmnb)
{
    logInfo(String("Essai profil LTE: ") + name);

    _networkRegistered = false;
    _packetAttached = false;

    {
        // AT+CMNB: 1=LTE-M, 2=NB-IoT, 3=both (replaces BG77 AT+QCFG="iotopmode")
        String cmd = "AT+CMNB=";
        cmd += String(cmnb);

        if (!sendATOK(cmd, 5000))
        {
            logWarn(String("CMNB refuse pour ") + name);
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

    // Signal quality via AT+CPSI? (replaces BG77 AT+QCSQ)
    // Response: +CPSI: LTE NB,Online,MCC-MNC,TAC,CID,BAND,EARFCN,...,RSRP,RSRQ,SINR,...
    String cpsiResp;
    sendATWaitFor("AT+CPSI?", "+CPSI:", nullptr, 5000, &cpsiResp);
    int cpsiIdx = cpsiResp.indexOf("+CPSI:");
    if (cpsiIdx >= 0)
    {
        String cpsiLine = cpsiResp.substring(cpsiIdx + 6);
        cpsiLine.trim();
        // Skip first two fields (type and "Online") to get signal data
        int c1 = cpsiLine.indexOf(',');
        int c2 = (c1 >= 0) ? cpsiLine.indexOf(',', c1 + 1) : -1;
        if (c2 >= 0)
        {
            String sigPart = cpsiLine.substring(c2 + 1);
            sigPart.replace(",", ";");
            if (sigPart.length() > 0)
            {
                _lteStatus += ";";
                _lteStatus += sigPart;
            }
        }
    }

    sendATWaitFor("AT+COPS?", "+COPS:", nullptr, 5000, nullptr);

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

void Sim7080Mqtt::mqttDisconnect()
{
    sendATOK("AT+SMDISC", 5000);
    _mqttConnected = false;
    printStateSnapshot("DISCONNECT");
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

bool Sim7080Mqtt::init()
{
    logInfo("Init complete LTE fallback + MQTT SIM7080G");

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

    if (!_networkRegistered)
    {
        sendATWaitFor("ATI", "OK", "ERROR", 5000, nullptr);

        bool registered = false;

        // cmnb: 2=NB-IoT, 1=LTE-M (replaces BG77 iotopmode 1=NB-IoT, 0=Cat-M1)
        registered = tryNetworkProfile("Bouygues NB-IoT", PLMN_BOUYGUES, ACT_NBIOT, 2);

        if (!registered)
        {
            logWarn("Echec Bouygues NB-IoT -> power cycle SIM7080G");
            modemPowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Orange NB-IoT", PLMN_ORANGE, ACT_NBIOT, 2);

        if (!registered)
        {
            logWarn("Echec Orange NB-IoT -> power cycle SIM7080G");
            modemPowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Bouygues Cat-M1", PLMN_BOUYGUES, ACT_CATM1, 1);

        if (!registered)
        {
            logWarn("Echec Bouygues Cat-M1 -> power cycle SIM7080G");
            modemPowerCycle();
        }

        if (!registered)
            registered = tryNetworkProfile("Orange Cat-M1", PLMN_ORANGE, ACT_CATM1, 1);

        if (!registered)
        {
            logWarn("Echec Orange Cat-M1 -> power cycle SIM7080G");
            modemPowerCycle();
            logError("Aucun profil LTE disponible");
            return false;
        }
    }
    else
    {
        logInfo("Deja enregistre, reconnect direct");

        String cpsiResp;
        sendATWaitFor("AT+CPSI?", "+CPSI:", nullptr, 5000, &cpsiResp);
        int cpsiIdx = cpsiResp.indexOf("+CPSI:");
        if (cpsiIdx >= 0)
        {
            String cpsiLine = cpsiResp.substring(cpsiIdx + 6);
            cpsiLine.trim();
            int c1 = cpsiLine.indexOf(',');
            int c2 = (c1 >= 0) ? cpsiLine.indexOf(',', c1 + 1) : -1;
            if (c2 >= 0)
            {
                int s1 = _lteStatus.indexOf(';');
                int s2 = (s1 >= 0) ? _lteStatus.indexOf(';', s1 + 1) : -1;
                String base = (s2 >= 0) ? _lteStatus.substring(0, s2) : _lteStatus;
                String sigPart = cpsiLine.substring(c2 + 1);
                sigPart.replace(",", ";");
                if (sigPart.length() > 0)
                    _lteStatus = base + ";" + sigPart;
            }
        }

        sendATWaitFor("AT+COPS?", "+COPS:", nullptr, 5000, nullptr);
    }

    {
        // AT+CGDCONT: standard 3GPP APN config (also present in SIM7080G APN manual config §4.2)
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

    // AT+CNCFG + AT+CNACT: SIM7080G PDP context activation (mandatory, replaces BG77 CGPADDR)
    {
        String cmd = "AT+CNCFG=0,1,\"";
        cmd += APN_NAME;
        cmd += "\"";
        sendATOK(cmd, 5000); // best-effort: may already be configured
    }

    // Activate PDP context 0; OK comes immediately, +APP PDP: 0,ACTIVE follows as URC.
    // If the context is stuck in an old state, deactivate once and retry before failing.
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
    pumpModem(2000); // drain +APP PDP: 0,ACTIVE URC

    if (!hasIPAddress())
    {
        logError("Pas d'adresse IP");
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
