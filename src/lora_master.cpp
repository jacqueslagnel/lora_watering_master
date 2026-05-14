/**
 * @file lora_master.cpp
 * @brief Implementation of the LoRa P2P master protocol controller.
 *
 * This file implements LoRaMaster, including SX126x radio initialization,
 * frame construction and validation, node presence tracking, queued and
 * scheduled command delivery, command acknowledgment handling, and MQTT-style
 * result payload publishing.
 */
#include "lora_master.h"
#include "ble_ota.h"

#ifdef _VARIANT_RAK4630_
#include <Adafruit_TinyUSB.h>
#endif

#include <SX126x-Arduino.h>
#include <SPI.h>
#include <string.h>

/**
 * @brief Global radio callback table passed to the SX126x driver.
 */
static RadioEvents_t g_radioEvents;
/**
 * @brief Active LoRaMaster instance used by static radio callback bridges.
 */
LoRaMaster *LoRaMaster::_instance = nullptr;

/**
 * @brief Constructs a LoRaMaster with cleared buffers, empty node table, and default schedule.
 */
LoRaMaster::LoRaMaster()
    : _masterBatteryReader(nullptr),
      _masterRebootHandler(nullptr),
      _publishCallback(nullptr),
      _rxDoneFlag(false),
      _txDoneFlag(false),
      _txTimeoutFlag(false),
      _rxSize(0),
      _lastRxRssi(0),
      _lastRxSnr(0),
      _masterState(STATE_RX),
      _currentHelloNodeId(0),
      _currentHelloSeq(0),
      _pendingCmdAfterAck(false),
      _pendingCmdTargetNode(0),
      _pendingCmdSeq(0),
      _pendingCmdCode(CMD_NONE),
      _pendingCmdArgU16(0),
      _pendingCmdIsQueued(false),
      _pendingCmdIsScheduled(false),
      _awaitingAckNodeId(0),
      _awaitingAckSeq(0),
      _awaitingAckCmd(CMD_NONE),
      _awaitingAckStartedMs(0)
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    memset(_txBuffer, 0, sizeof(_txBuffer));
    memset(_pendingCmdText, 0, sizeof(_pendingCmdText));
    memset(_nodes, 0, sizeof(_nodes));

    for (uint8_t i = 0; i <= MAX_NODES; i++)
    {
        _queuedCmds[i].active = false;
        _queuedCmds[i].awaitingAck = false;
        _queuedCmds[i].targetNode = 0;
        _queuedCmds[i].cmdCode = CMD_NONE;
        _queuedCmds[i].argU16 = 0;
        _queuedCmds[i].text[0] = '\0';
        _queuedCmds[i].mqttVerb[0] = '\0';
        _queuedCmds[i].mqttArgText[0] = '\0';
        _queuedCmds[i].lastSeq = 0;
        _queuedCmds[i].retriesDone = 0;
        _queuedCmds[i].maxRetries = MAX_CMD_RETRIES;
    }

    _schedule.enabled = true;
    _schedule.cycleActive = false;
    _schedule.periodMs = DEFAULT_BATTERY_SCHEDULE_MS;
    _schedule.nextDueMs = 0;
    _schedule.cycleId = 0;
}

void LoRaMaster::setPublishCallback(PublishCallback cb)
{
    _publishCallback = cb;
}

uint8_t LoRaMaster::getOnlineNodeCount() const
{
    // FIX: utilise _nodes[i].online (mis à jour par refreshOnlineStates)
    // au lieu de recalculer le timeout inline, évitant une incohérence
    // entre les deux chemins de détection.
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (_nodes[i].used && _nodes[i].online)
        {
            count++;
        }
    }
    return count;
}

void LoRaMaster::setBatterySchedulePeriodSec(uint32_t seconds)
{
    if (seconds == 0)
    {
        return;
    }

    _schedule.periodMs = seconds * 1000UL;
    _schedule.nextDueMs = millis() + _schedule.periodMs;
    _schedule.cycleActive = false;
}

void LoRaMaster::begin()
{
    _instance = this;
    initRadio();
    _schedule.nextDueMs = millis() + _schedule.periodMs;

    printTsvHeader();
    printTsv("BOOT", 0, 0, 0, 0, 0, 0, 0, "lora_master_start");
    printTsv("SCHED_INIT", 0, 0, 0, 0, 0, 0, CMD_BATTERY, "battery_all_nodes_every_5min");

    startContinuousRx();
}

void LoRaMaster::loop()
{
    Radio.IrqProcess();
    refreshOnlineStates();
    maybeStartScheduleCycle();
    handleAckTimeouts();

    if (_rxDoneFlag)
    {
        _rxDoneFlag = false;
        processReceivedFrame();
    }

    if (_txDoneFlag)
    {
        _txDoneFlag = false;
        handleTxDone();
    }

    if (_txTimeoutFlag)
    {
        _txTimeoutFlag = false;
        handleTxTimeout();
    }
}

void LoRaMaster::handleConsoleLine(const String &line)
{
    String s = line;
    s.trim();
    if (s.length() == 0)
    {
        return;
    }

    String lower = s;
    lower.toLowerCase();

    if (lower == "list")
    {
        dumpNodeListTsv();
        return;
    }

    if (lower == "clear")
    {
        for (uint8_t i = 1; i <= MAX_NODES; i++)
        {
            clearQueuedCommand(i);
        }
        printTsv("CMD_CLEAR", 0, 0, 0, 0, 0, 0, 0, "cleared");
        return;
    }

    if (lower.startsWith("schedule "))
    {
        uint32_t sec = (uint32_t)lower.substring(9).toInt();
        if (sec > 0)
        {
            setBatterySchedulePeriodSec(sec);
            printTsv("SCHED_SET", 0, 0, 0, 0, 0, 0, CMD_BATTERY, "period_updated");
        }
        return;
    }

    if (lower.startsWith("mqtt "))
    {
        handleMqttCommand(s.substring(5));
        return;
    }

    printTsv("SERIAL_ERR", 0, 0, 0, 0, 0, 0, 0, "unknown_lora_cmd");
}

bool LoRaMaster::handleMqttCommand(const String &payload)
{
    String masterVerb;
    String masterArg;

    if (parseMqttMasterCommand(payload, masterVerb, masterArg))
    {

        if (masterVerb == "ota")
        {
            uint32_t sec = BleOta::sanitizeDurationSec((uint32_t)masterArg.toInt());
            publishMasterOtaResult(true, sec);
            delay(100);
            BleOta::requestBoot(sec);
            NVIC_SystemReset();
            return true;
        }

        if (masterVerb == "list")
        {
            publishMasterListResult(true);
            dumpNodeListTsv();
            return true;
        }

        if (masterVerb == "schedule")
        {
            uint32_t sec = (uint32_t)masterArg.toInt();
            if (sec == 0)
            {
                publishMasterScheduleResult(false, 0);
                return false;
            }

            setBatterySchedulePeriodSec(sec);
            publishMasterScheduleResult(true, sec);
            return true;
        }

        if (masterVerb == "bat" || masterVerb == "battery")
        {
            if (_masterBatteryReader == nullptr)
            {
                publishPayload("M;bat;0;no_reader");
                return false;
            }

            uint16_t mv = _masterBatteryReader();

            String response = "M;bat;1;";
            response += String(mv);

            publishPayload(response);
            return true;
        }

        if (masterVerb == "reboot")
        {
            if (_masterRebootHandler == nullptr)
            {
                publishPayload("M;reboot;0;no_handler");
                return false;
            }

            publishPayload("M;reboot;1");
            delay(500);
            _masterRebootHandler();
            return true;
        }

        String response = "M;error;unknown_cmd;";
        response += masterVerb;
        publishPayload(response);
        return false;
    }

    uint8_t nodeId = 0;
    uint8_t cmdCode = CMD_NONE;
    uint16_t argU16 = 0;
    String textArg;
    String verb;
    String argText;

    if (!parseMqttNodeCommand(payload, nodeId, cmdCode, argU16, textArg, verb, argText))
    {
        if (nodeId > 0)
        {
            publishQueuedCommandFailure(nodeId, cmdCode, argU16);
        }
        else
        {
            publishInvalidNodeCommandResult(payload);
        }
        return false;
    }

    if (nodeId == 0 || nodeId > MAX_NODES)
    {
        publishQueuedCommandFailure(nodeId, cmdCode, argU16);
        return false;
    }

    if (hasQueuedCommand(nodeId))
    {
        publishQueuedCommandFailure(nodeId, cmdCode, argU16);
        return false;
    }

    if (!queueNodeCommand(nodeId, cmdCode, argU16, textArg.c_str(), verb.c_str(), argText.c_str()))
    {
        publishQueuedCommandFailure(nodeId, cmdCode, argU16);
        return false;
    }

    return true;
}

void LoRaMaster::initRadio()
{
#ifdef _VARIANT_RAK4630_
    lora_rak4630_init();
#endif

    g_radioEvents.TxDone = LoRaMaster::onTxDone;
    g_radioEvents.TxTimeout = LoRaMaster::onTxTimeout;
    g_radioEvents.RxDone = LoRaMaster::onRxDone;

    Radio.Init(&g_radioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD,
        true,
        0,
        0,
        LORA_IQ_INVERSION,
        TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0,
        LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD,
        0,
        true,
        0,
        0,
        LORA_IQ_INVERSION,
        true);
}

void LoRaMaster::resetFlags()
{
    _rxDoneFlag = false;
    _txDoneFlag = false;
    _txTimeoutFlag = false;
    _rxSize = 0;
}

void LoRaMaster::startContinuousRx()
{
    _masterState = STATE_RX;
    Radio.Rx(0);
    printTsv("RX_ON", 0, 0, 0, 0, 0, 0, 0, "continuous");
}

uint16_t LoRaMaster::crc16Ccitt(const uint8_t *data, uint16_t len) const
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t LoRaMaster::getFrameTotalLength(uint8_t payloadLen) const
{
    return (uint8_t)(4 + payloadLen + 2);
}

uint8_t LoRaMaster::buildFrame(uint8_t type, uint8_t nodeId, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    _txBuffer[0] = type;
    _txBuffer[1] = nodeId;
    _txBuffer[2] = seq;
    _txBuffer[3] = len;

    for (uint8_t i = 0; i < len; i++)
    {
        _txBuffer[4 + i] = payload[i];
    }

    uint16_t crc = crc16Ccitt(_txBuffer, (uint16_t)(4 + len));
    _txBuffer[4 + len] = (uint8_t)(crc & 0xFF);
    _txBuffer[5 + len] = (uint8_t)((crc >> 8) & 0xFF);

    return getFrameTotalLength(len);
}

bool LoRaMaster::validateReceivedFrame() const
{
    if (_rxSize < 6)
    {
        return false;
    }

    uint8_t payloadLen = _rxBuffer[3];
    uint16_t expectedTotal = (uint16_t)(4 + payloadLen + 2);
    if (_rxSize != expectedTotal)
    {
        return false;
    }

    uint16_t rxCrc = (uint16_t)_rxBuffer[4 + payloadLen] |
                     ((uint16_t)_rxBuffer[5 + payloadLen] << 8);
    uint16_t calcCrc = crc16Ccitt(_rxBuffer, (uint16_t)(4 + payloadLen));

    return (rxCrc == calcCrc);
}

void LoRaMaster::printFrameDebug(const char *prefix, const uint8_t *buf, uint8_t len) const
{
    Serial.print(prefix);
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" data=");
    for (uint8_t i = 0; i < len; i++)
    {
        if (buf[i] < 16)
        {
            Serial.print('0');
        }
        Serial.print(buf[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void LoRaMaster::printTsvHeader()
{
    Serial.println("timestamp_ms\tevent\tnode_id\tseq\tbattery_mV\tstatus\trssi\tsnr\tcmd\tinfo");
}

void LoRaMaster::printTsv(const char *eventName,
                          uint8_t nodeId,
                          uint8_t seq,
                          uint16_t battery_mV,
                          uint8_t statusFlag,
                          int16_t rssi,
                          int8_t snr,
                          uint8_t cmd,
                          const char *info)
{
    Serial.print(millis());
    Serial.print('\t');
    Serial.print(eventName);
    Serial.print('\t');
    Serial.print(nodeId);
    Serial.print('\t');
    Serial.print(seq);
    Serial.print('\t');
    Serial.print(battery_mV);
    Serial.print('\t');
    Serial.print(statusFlag);
    Serial.print('\t');
    Serial.print(rssi);
    Serial.print('\t');
    Serial.print((int)snr);
    Serial.print('\t');
    Serial.print(cmd);
    Serial.print('\t');
    Serial.println(info);
}

int LoRaMaster::findNodeIndex(uint8_t nodeId) const
{
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (_nodes[i].used && _nodes[i].nodeId == nodeId)
        {
            return i;
        }
    }
    return -1;
}

int LoRaMaster::allocateNodeIndex(uint8_t nodeId)
{
    int idx = findNodeIndex(nodeId);
    if (idx >= 0)
    {
        return idx;
    }

    for (int i = 0; i < MAX_NODES; i++)
    {
        if (!_nodes[i].used)
        {
            _nodes[i].used = true;
            _nodes[i].online = false;
            _nodes[i].notifiedOnline = false;
            _nodes[i].notifiedLost = false;
            _nodes[i].nodeId = nodeId;
            _nodes[i].lastHelloSeq = 0;
            _nodes[i].statusFlag = 0;
            _nodes[i].lastBattery_mV = 0;
            _nodes[i].lastRssi = 0;
            _nodes[i].lastSnr = 0;
            _nodes[i].lastSeenMs = 0;
            _nodes[i].lastCycleIdHandled = 0;
            _nodes[i].lastValveOpen = 0;
            _nodes[i].lastWateringActive = 0;
            _nodes[i].lastWaterDurationSec = 0;
            _nodes[i].lastWaterRemainingSec = 0;
            _nodes[i].lastFlowPulses = 0;
            _nodes[i].lastLitres = 0;
            return i;
        }
    }

    return -1;
}

void LoRaMaster::updateNodeHello(uint8_t nodeId, uint8_t seq, uint8_t statusFlag, int16_t rssi, int8_t snr)
{
    int idx = allocateNodeIndex(nodeId);
    if (idx < 0)
    {
        return;
    }

    bool wasOnline = _nodes[idx].online;

    _nodes[idx].used = true;
    _nodes[idx].online = true;
    _nodes[idx].nodeId = nodeId;
    _nodes[idx].lastHelloSeq = seq;
    _nodes[idx].statusFlag = statusFlag;
    _nodes[idx].lastRssi = rssi;
    _nodes[idx].lastSnr = snr;
    _nodes[idx].lastSeenMs = millis();

    if (!wasOnline || !_nodes[idx].notifiedOnline)
    {
        publishNodePresence(nodeId, true);
        _nodes[idx].notifiedOnline = true;
        _nodes[idx].notifiedLost = false;
    }
}

void LoRaMaster::updateNodeBattery(uint8_t nodeId, uint16_t battery_mV)
{
    int idx = allocateNodeIndex(nodeId);
    if (idx >= 0)
    {
        _nodes[idx].lastBattery_mV = battery_mV;
    }
}

void LoRaMaster::updateNodeStatus(uint8_t nodeId,
                                  uint16_t battery_mV,
                                  uint8_t statusFlag,
                                  uint32_t uptimeSec,
                                  uint16_t rssiAbs,
                                  int8_t snr)
{
    int idx = allocateNodeIndex(nodeId);
    if (idx < 0)
    {
        return;
    }

    _nodes[idx].lastBattery_mV = battery_mV;
    _nodes[idx].statusFlag = statusFlag;
    _nodes[idx].lastRssi = -(int16_t)rssiAbs;
    _nodes[idx].lastSnr = snr;
}

void LoRaMaster::updateNodeWaterStatus(uint8_t nodeId,
                                       uint8_t valveOpen,
                                       uint8_t wateringActive,
                                       uint16_t waterDurationSec,
                                       uint16_t remainingSec,
                                       uint32_t flowPulses,
                                       uint16_t litres)
{
    int idx = allocateNodeIndex(nodeId);
    if (idx < 0)
    {
        return;
    }

    _nodes[idx].lastValveOpen = valveOpen;
    _nodes[idx].lastWateringActive = wateringActive;
    _nodes[idx].lastWaterDurationSec = waterDurationSec;
    _nodes[idx].lastWaterRemainingSec = remainingSec;
    _nodes[idx].lastFlowPulses = flowPulses;
    _nodes[idx].lastLitres = litres;
}

void LoRaMaster::refreshOnlineStates()
{
    uint32_t now = millis();

    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (!_nodes[i].used)
        {
            continue;
        }

        bool isNowOnline = ((now - _nodes[i].lastSeenMs) <= NODE_ONLINE_TIMEOUT_MS);

        if (_nodes[i].online && !isNowOnline)
        {
            _nodes[i].online = false;

            PendingNodeCommand *queued = getQueuedCommand(_nodes[i].nodeId);
            if (queued != nullptr && queued->active)
            {
                printTsv("CMD_DROP", _nodes[i].nodeId, 0, 0, 0, 0, 0, queued->cmdCode, "node_offline");
                publishQueuedCommandFailure(_nodes[i].nodeId, queued->cmdCode, queued->argU16);
                clearQueuedCommand(_nodes[i].nodeId);
            }

            if (_schedule.cycleActive)
            {
                _nodes[i].lastCycleIdHandled = _schedule.cycleId;
            }

            if (!_nodes[i].notifiedLost)
            {
                publishNodePresence(_nodes[i].nodeId, false);
                _nodes[i].notifiedLost = true;
                _nodes[i].notifiedOnline = false;
            }
        }
        else if (!_nodes[i].online && isNowOnline)
        {
            _nodes[i].online = true;
            if (!_nodes[i].notifiedOnline)
            {
                publishNodePresence(_nodes[i].nodeId, true);
                _nodes[i].notifiedOnline = true;
                _nodes[i].notifiedLost = false;
            }
        }
    }

    if (_schedule.cycleActive && isScheduleCycleComplete())
    {
        _schedule.cycleActive = false;
        _schedule.nextDueMs = millis() + _schedule.periodMs;
        printTsv("SCHED_CYCLE_DONE", 0, 0, 0, 0, 0, 0, CMD_BATTERY, "battery_all_nodes");
    }
}

void LoRaMaster::dumpNodeListTsv()
{
    refreshOnlineStates();

    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (!_nodes[i].used)
        {
            continue;
        }

        printTsv("NODE",
                 _nodes[i].nodeId,
                 _nodes[i].lastHelloSeq,
                 _nodes[i].lastBattery_mV,
                 _nodes[i].statusFlag,
                 _nodes[i].lastRssi,
                 _nodes[i].lastSnr,
                 0,
                 _nodes[i].online ? "online" : "offline");
    }
}

LoRaMaster::PendingNodeCommand *LoRaMaster::getQueuedCommand(uint8_t nodeId)
{
    if (nodeId == 0 || nodeId > MAX_NODES)
    {
        return nullptr;
    }
    return &_queuedCmds[nodeId];
}

const LoRaMaster::PendingNodeCommand *LoRaMaster::getQueuedCommand(uint8_t nodeId) const
{
    if (nodeId == 0 || nodeId > MAX_NODES)
    {
        return nullptr;
    }
    return &_queuedCmds[nodeId];
}

bool LoRaMaster::hasQueuedCommand(uint8_t nodeId) const
{
    const PendingNodeCommand *cmd = getQueuedCommand(nodeId);
    return (cmd != nullptr) ? cmd->active : false;
}

void LoRaMaster::clearQueuedCommand(uint8_t nodeId)
{
    PendingNodeCommand *cmd = getQueuedCommand(nodeId);
    if (cmd == nullptr)
    {
        return;
    }

    cmd->active = false;
    cmd->awaitingAck = false;
    cmd->targetNode = 0;
    cmd->cmdCode = CMD_NONE;
    cmd->argU16 = 0;
    cmd->text[0] = '\0';
    cmd->mqttVerb[0] = '\0';
    cmd->mqttArgText[0] = '\0';
    cmd->lastSeq = 0;
    cmd->retriesDone = 0;
    cmd->maxRetries = MAX_CMD_RETRIES;
}

bool LoRaMaster::queueNodeCommand(uint8_t nodeId,
                                  uint8_t cmdCode,
                                  uint16_t argU16,
                                  const char *text,
                                  const char *mqttVerb,
                                  const char *mqttArgText)
{
    PendingNodeCommand *cmd = getQueuedCommand(nodeId);
    if (cmd == nullptr)
    {
        return false;
    }

    cmd->active = true;
    cmd->awaitingAck = true;
    cmd->targetNode = nodeId;
    cmd->cmdCode = cmdCode;
    cmd->argU16 = argU16;
    cmd->lastSeq = 0;
    cmd->retriesDone = 0;
    cmd->maxRetries = MAX_CMD_RETRIES;

    if (text != nullptr)
    {
        strncpy(cmd->text, text, sizeof(cmd->text) - 1);
        cmd->text[sizeof(cmd->text) - 1] = '\0';
    }
    else
    {
        cmd->text[0] = '\0';
    }

    if (mqttVerb != nullptr)
    {
        strncpy(cmd->mqttVerb, mqttVerb, sizeof(cmd->mqttVerb) - 1);
        cmd->mqttVerb[sizeof(cmd->mqttVerb) - 1] = '\0';
    }
    else
    {
        cmd->mqttVerb[0] = '\0';
    }

    if (mqttArgText != nullptr)
    {
        strncpy(cmd->mqttArgText, mqttArgText, sizeof(cmd->mqttArgText) - 1);
        cmd->mqttArgText[sizeof(cmd->mqttArgText) - 1] = '\0';
    }
    else
    {
        cmd->mqttArgText[0] = '\0';
    }

    return true;
}

void LoRaMaster::clearPendingContext()
{
    _pendingCmdAfterAck = false;
    _pendingCmdTargetNode = 0;
    _pendingCmdSeq = 0;
    _pendingCmdCode = CMD_NONE;
    _pendingCmdArgU16 = 0;
    _pendingCmdText[0] = '\0';
    _pendingCmdIsQueued = false;
    _pendingCmdIsScheduled = false;
}

void LoRaMaster::maybeStartScheduleCycle()
{
    if (!_schedule.enabled || _schedule.cycleActive)
    {
        return;
    }

    // BUG des 49 jours :if (millis() < _schedule.nextDueMs)
    // FIX
    if ((int32_t)(millis() - _schedule.nextDueMs) < 0)
    {
        return;
    }

    _schedule.cycleActive = true;
    _schedule.cycleId++;
    printTsv("SCHED_CYCLE_START", 0, 0, 0, 0, 0, 0, CMD_BATTERY, "battery_all_nodes");
}

bool LoRaMaster::hasScheduleWorkForNode(uint8_t nodeId) const
{
    if (!_schedule.enabled || !_schedule.cycleActive)
    {
        return false;
    }

    int idx = findNodeIndex(nodeId);
    if (idx < 0 || !_nodes[idx].online)
    {
        return false;
    }

    return (_nodes[idx].lastCycleIdHandled != _schedule.cycleId);
}

void LoRaMaster::markScheduleDoneForNode(uint8_t nodeId)
{
    int idx = findNodeIndex(nodeId);
    if (idx >= 0)
    {
        _nodes[idx].lastCycleIdHandled = _schedule.cycleId;
    }

    if (isScheduleCycleComplete())
    {
        _schedule.cycleActive = false;
        _schedule.nextDueMs = millis() + _schedule.periodMs;
        printTsv("SCHED_CYCLE_DONE", 0, 0, 0, 0, 0, 0, CMD_BATTERY, "battery_all_nodes");
    }
}

bool LoRaMaster::isScheduleCycleComplete() const
{
    if (!_schedule.cycleActive)
    {
        return true;
    }

    // bool anyOnline = false;
    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (_nodes[i].used && _nodes[i].online)
        {
            // anyOnline = true;
            if (_nodes[i].lastCycleIdHandled != _schedule.cycleId)
            {
                return false;
            }
        }
    }

    // FIX: si aucun nœud n'est en ligne, le cycle est considéré terminé
    // (avant: retournait 'anyOnline' = false → cycle bloqué indéfiniment)
    return true;
}

bool LoRaMaster::sendFrame(uint8_t totalLen)
{
    resetFlags();
    printFrameDebug("TX", _txBuffer, totalLen);
    Radio.Send(_txBuffer, totalLen);
    return true;
}

void LoRaMaster::sendAckHello(uint8_t nodeId, uint8_t seq)
{
    uint8_t totalLen = buildFrame(FRAME_ACK_HELLO, nodeId, seq, nullptr, 0);
    _currentHelloNodeId = nodeId;
    _currentHelloSeq = seq;
    printTsv("ACK_HELLO_TX", nodeId, seq, 0, 0, 0, 0, 0, "start");
    sendFrame(totalLen);
    _masterState = STATE_WAIT_ACK_HELLO_TXDONE;
}

void LoRaMaster::sendCmdNow(uint8_t nodeId, uint8_t seq, uint8_t cmdCode, uint16_t argU16, const char *textPayload)
{
    uint8_t payload[64];
    uint8_t len = 1;
    payload[0] = cmdCode;

    if (cmdCode == CMD_SET_PERIOD || cmdCode == CMD_WATER_TIME)
    {
        payload[len++] = (uint8_t)(argU16 & 0xFF);
        payload[len++] = (uint8_t)((argU16 >> 8) & 0xFF);
    }
    else if (cmdCode == CMD_TEXT_MESSAGE && textPayload != nullptr)
    {
        // Slave rxBuffer = 64 bytes; max frame = 64: header(4) + cmdCode(1) + text(57) + CRC(2)
        static constexpr uint8_t MAX_TEXT_LEN = 57;
        uint8_t textLen = (uint8_t)strlen(textPayload);
        if (textLen > MAX_TEXT_LEN)
        {
            textLen = MAX_TEXT_LEN;
        }
        memcpy(&payload[1], textPayload, textLen);
        len += textLen;
    }

    uint8_t totalLen = buildFrame(FRAME_CMD, nodeId, seq, payload, len);
    printTsv("CMD_TX", nodeId, seq, 0, 0, 0, 0, cmdCode, "send");
    sendFrame(totalLen);
    _masterState = STATE_WAIT_CMD_TXDONE;
}

void LoRaMaster::processHelloFrame()
{
    if (_rxSize < 7)
    {
        printTsv("HELLO_ERR", 0, 0, 0, 0, _lastRxRssi, _lastRxSnr, 0, "too_short");
        startContinuousRx();
        return;
    }

    uint8_t nodeId = _rxBuffer[1];
    uint8_t seq = _rxBuffer[2];
    uint8_t payloadLen = _rxBuffer[3];

    if (payloadLen < 1)
    {
        printTsv("HELLO_ERR", nodeId, seq, 0, 0, _lastRxRssi, _lastRxSnr, 0, "bad_len");
        startContinuousRx();
        return;
    }

    uint8_t helloFlags = _rxBuffer[4];
    bool bootFlag = (helloFlags & 0x80) != 0;
    uint8_t statusFlag = (helloFlags & 0x01) ? 1 : 0;

    updateNodeHello(nodeId, seq, statusFlag, _lastRxRssi, _lastRxSnr);
    printTsv("HELLO_RX", nodeId, seq, 0, statusFlag, _lastRxRssi, _lastRxSnr, 0, "ok");
    if (bootFlag)
    {
        publishNodeBootResult(nodeId);
    }
    clearPendingContext();

    PendingNodeCommand *queued = getQueuedCommand(nodeId);
    if (queued != nullptr && queued->active && queued->awaitingAck)
    {
        if (queued->retriesDone >= queued->maxRetries)
        {
            printTsv("CMD_DROP", nodeId, seq, 0, statusFlag, _lastRxRssi, _lastRxSnr, queued->cmdCode, "queued_max_retries");
            publishQueuedCommandFailure(nodeId, queued->cmdCode, queued->argU16);
            clearQueuedCommand(nodeId);
        }
        else
        {
            _pendingCmdAfterAck = true;
            _pendingCmdTargetNode = nodeId;
            _pendingCmdSeq = seq;
            _pendingCmdCode = queued->cmdCode;
            _pendingCmdArgU16 = queued->argU16;
            strncpy(_pendingCmdText, queued->text, sizeof(_pendingCmdText) - 1);
            _pendingCmdText[sizeof(_pendingCmdText) - 1] = '\0';
            _pendingCmdIsQueued = true;
            printTsv("CMD_WINDOW", nodeId, seq, 0, statusFlag, _lastRxRssi, _lastRxSnr, queued->cmdCode, queued->mqttVerb);
        }
    }
    else if (hasScheduleWorkForNode(nodeId))
    {
        _pendingCmdAfterAck = true;
        _pendingCmdTargetNode = nodeId;
        _pendingCmdSeq = seq;
        _pendingCmdCode = CMD_BATTERY;
        _pendingCmdArgU16 = 0;
        _pendingCmdText[0] = '\0';
        _pendingCmdIsScheduled = true;
        printTsv("SCHED_WINDOW", nodeId, seq, 0, statusFlag, _lastRxRssi, _lastRxSnr, CMD_BATTERY, "battery_request");
    }

    sendAckHello(nodeId, seq);
}

void LoRaMaster::processAckCmdFrame()
{
    if (_rxSize < 3)
    {
        printTsv("ACK_CMD_ERR", 0, 0, 0, 0,
                 _lastRxRssi, _lastRxSnr, 0, "too_short_hdr");
        startContinuousRx();
        return;
    }

    uint8_t nodeId = _rxBuffer[1];
    uint8_t seq = _rxBuffer[2];

    if (_rxSize < 8)
    {
        printTsv("ACK_CMD_ERR", nodeId, seq, 0, 0,
                 _lastRxRssi, _lastRxSnr, 0, "too_short");

        startContinuousRx();
        return;
    }

    uint8_t payloadLen = _rxBuffer[3];
    uint8_t ackedCmd = _rxBuffer[4];
    uint8_t ackStatus = _rxBuffer[5];

    if (_awaitingAckNodeId == 0)
    {
        printTsv("ACK_CMD_IGN", nodeId, seq, 0, 0,
                 _lastRxRssi, _lastRxSnr, ackedCmd, "no_ack_expected");
        startContinuousRx();
        return;
    }

    if (_awaitingAckNodeId != nodeId ||
        _awaitingAckSeq != seq ||
        _awaitingAckCmd != ackedCmd)
    {
        printTsv("ACK_CMD_IGN", nodeId, seq, 0, 0,
                 _lastRxRssi, _lastRxSnr, ackedCmd, "unexpected_ack");
        startContinuousRx();
        return;
    }

    bool ok = (ackStatus == ACK_STATUS_OK);

    PendingNodeCommand *queued = getQueuedCommand(nodeId);

    if (ackedCmd == CMD_BATTERY)
    {
        if (ok && payloadLen >= 4)
        {
            uint16_t batteryMv = (uint16_t)_rxBuffer[6] | ((uint16_t)_rxBuffer[7] << 8);
            updateNodeBattery(nodeId, batteryMv);
            printTsv("ACK_CMD_RX", nodeId, seq, batteryMv, 0,
                     _lastRxRssi, _lastRxSnr, ackedCmd, "battery_ok");
            publishBatteryResult(nodeId, true, batteryMv, 0, _lastRxRssi, _lastRxSnr);
        }
        else if (!_pendingCmdIsScheduled)
        {
            publishBatteryResult(nodeId, false, 0, 0, 0, 0);
        }

        if (_schedule.cycleActive)
        {
            markScheduleDoneForNode(nodeId);
        }
    }
    else if (ackedCmd == CMD_STATUS)
    {
        if (ok && payloadLen >= 9)
        {
            uint16_t batteryMv = (uint16_t)_rxBuffer[6] | ((uint16_t)_rxBuffer[7] << 8);
            uint8_t statusFlag = _rxBuffer[8];
            uint32_t uptimeSec = (uint32_t)_rxBuffer[9] |
                                 ((uint32_t)_rxBuffer[10] << 8) |
                                 ((uint32_t)_rxBuffer[11] << 16) |
                                 ((uint32_t)_rxBuffer[12] << 24);

            updateNodeStatus(nodeId, batteryMv, statusFlag, uptimeSec,
                             (uint16_t)abs(_lastRxRssi), _lastRxSnr);

            printTsv("ACK_CMD_RX", nodeId, seq, batteryMv, statusFlag,
                     _lastRxRssi, _lastRxSnr, ackedCmd, "status_ok");

            publishStatusResult(nodeId, true, batteryMv, statusFlag,
                                uptimeSec, _lastRxRssi, _lastRxSnr);
        }
        else
        {
            publishStatusResult(nodeId, false, 0, 0, 0, 0, 0);
        }
    }
    else if (ackedCmd == CMD_WATER_STATUS)
    {
        if (ok && payloadLen >= 19)
        {
            uint8_t valveOpen = _rxBuffer[6];
            uint8_t wateringActive = _rxBuffer[7];
            uint16_t durationSec = (uint16_t)_rxBuffer[8] | ((uint16_t)_rxBuffer[9] << 8);
            uint16_t remainingSec = (uint16_t)_rxBuffer[10] | ((uint16_t)_rxBuffer[11] << 8);

            uint32_t flowPulses = (uint32_t)_rxBuffer[12] |
                                  ((uint32_t)_rxBuffer[13] << 8) |
                                  ((uint32_t)_rxBuffer[14] << 16) |
                                  ((uint32_t)_rxBuffer[15] << 24);

            uint16_t litres = (uint16_t)_rxBuffer[16] | ((uint16_t)_rxBuffer[17] << 8);

            uint32_t postClosePulses = (uint32_t)_rxBuffer[18] |
                                       ((uint32_t)_rxBuffer[19] << 8) |
                                       ((uint32_t)_rxBuffer[20] << 16) |
                                       ((uint32_t)_rxBuffer[21] << 24);

            uint8_t postCloseLeakDetected = _rxBuffer[22];

            updateNodeWaterStatus(nodeId, valveOpen, wateringActive,
                                  durationSec, remainingSec, flowPulses, litres);

            printTsv("ACK_CMD_RX", nodeId, seq, 0, 0,
                     _lastRxRssi, _lastRxSnr, ackedCmd, "wstatus_ok");

            publishWaterStatusResult(nodeId, true, valveOpen, wateringActive,
                                     durationSec, remainingSec, flowPulses, litres,
                                     postClosePulses, postCloseLeakDetected);
        }
        else
        {
            publishWaterStatusResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
    else if (ackedCmd == CMD_PING)
    {
        publishPingResult(nodeId, ok);
    }
    else if (ackedCmd == CMD_TEXT_MESSAGE)
    {
        publishTextResult(nodeId, ok);
    }
    else if (ackedCmd == CMD_REBOOT)
    {
        publishRebootResult(nodeId, ok);
    }
    else if (ackedCmd == CMD_SET_PERIOD)
    {
        uint16_t seconds = (queued != nullptr) ? queued->argU16 : 0;
        publishPeriodResult(nodeId, ok, seconds);
    }
    else if (ackedCmd == CMD_LED_ON || ackedCmd == CMD_LED_OFF)
    {
        uint8_t value = (ackedCmd == CMD_LED_ON) ? 1 : 0;
        publishLedResult(nodeId, ok, value);
    }
    else if (ackedCmd == CMD_VALVE_OPEN || ackedCmd == CMD_VALVE_CLOSE)
    {
        uint8_t value = (ackedCmd == CMD_VALVE_OPEN) ? 1 : 0;
        publishValveResult(nodeId, ok, value);
    }
    else if (ackedCmd == CMD_WATER_TIME)
    {
        if (ok && payloadLen >= 19)
        {
            uint8_t valveOpen = _rxBuffer[6];
            uint8_t wateringActive = _rxBuffer[7];
            uint16_t durationSec = (uint16_t)_rxBuffer[8] | ((uint16_t)_rxBuffer[9] << 8);
            uint16_t remainingSec = (uint16_t)_rxBuffer[10] | ((uint16_t)_rxBuffer[11] << 8);

            uint32_t flowPulses = (uint32_t)_rxBuffer[12] |
                                  ((uint32_t)_rxBuffer[13] << 8) |
                                  ((uint32_t)_rxBuffer[14] << 16) |
                                  ((uint32_t)_rxBuffer[15] << 24);

            uint16_t litres = (uint16_t)_rxBuffer[16] | ((uint16_t)_rxBuffer[17] << 8);

            uint32_t postClosePulses = (uint32_t)_rxBuffer[18] |
                                       ((uint32_t)_rxBuffer[19] << 8) |
                                       ((uint32_t)_rxBuffer[20] << 16) |
                                       ((uint32_t)_rxBuffer[21] << 24);

            uint8_t postCloseLeakDetected = _rxBuffer[22];

            updateNodeWaterStatus(nodeId, valveOpen, wateringActive,
                                  durationSec, remainingSec, flowPulses, litres);

            printTsv("ACK_CMD_RX", nodeId, seq, 0, 0,
                     _lastRxRssi, _lastRxSnr, ackedCmd, "water_ok");

            publishWaterStartResult(nodeId, true, valveOpen, wateringActive,
                                    durationSec, remainingSec, flowPulses, litres,
                                    postClosePulses, postCloseLeakDetected);
        }
        else
        {
            publishWaterStartResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
    else if (ackedCmd == CMD_WATER_ABORT)
    {
        if (ok && payloadLen >= 19)
        {
            uint8_t valveOpen = _rxBuffer[6];
            uint8_t wateringActive = _rxBuffer[7];
            uint16_t durationSec = (uint16_t)_rxBuffer[8] | ((uint16_t)_rxBuffer[9] << 8);
            uint16_t remainingSec = (uint16_t)_rxBuffer[10] | ((uint16_t)_rxBuffer[11] << 8);

            uint32_t flowPulses = (uint32_t)_rxBuffer[12] |
                                  ((uint32_t)_rxBuffer[13] << 8) |
                                  ((uint32_t)_rxBuffer[14] << 16) |
                                  ((uint32_t)_rxBuffer[15] << 24);

            uint16_t litres = (uint16_t)_rxBuffer[16] | ((uint16_t)_rxBuffer[17] << 8);

            uint32_t postClosePulses = (uint32_t)_rxBuffer[18] |
                                       ((uint32_t)_rxBuffer[19] << 8) |
                                       ((uint32_t)_rxBuffer[20] << 16) |
                                       ((uint32_t)_rxBuffer[21] << 24);

            uint8_t postCloseLeakDetected = _rxBuffer[22];

            updateNodeWaterStatus(nodeId, valveOpen, wateringActive,
                                  durationSec, remainingSec, flowPulses, litres);

            printTsv("ACK_CMD_RX", nodeId, seq, 0, 0,
                     _lastRxRssi, _lastRxSnr, ackedCmd, "abort_ok");

            publishWaterAbortResult(nodeId, true, valveOpen, wateringActive,
                                    durationSec, remainingSec, flowPulses, litres,
                                    postClosePulses, postCloseLeakDetected);
        }
        else
        {
            publishWaterAbortResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
    else if (ackedCmd == CMD_OTA_MODE)
    {
        publishNodeOtaResult(nodeId, ok);
    }
    else
    {
        printTsv("ACK_CMD_ERR", nodeId, seq, 0, 0,
                 _lastRxRssi, _lastRxSnr, ackedCmd, "unknown_cmd");
    }

    if (queued != nullptr &&
        queued->active &&
        queued->awaitingAck &&
        seq == queued->lastSeq &&
        ackedCmd == queued->cmdCode)
    {
        clearQueuedCommand(nodeId);
    }

    _awaitingAckNodeId = 0;
    _awaitingAckSeq = 0;
    _awaitingAckCmd = CMD_NONE;
    _awaitingAckStartedMs = 0;

    if (ok)
    {
        publishNodeSeen(nodeId);
    }

    startContinuousRx();
}

void LoRaMaster::processReceivedFrame()
{
    if (_rxSize < 6)
    {
        printTsv("RX_ERR", 0, 0, 0, 0, _lastRxRssi, _lastRxSnr, 0, "frame_too_short");
        startContinuousRx();
        return;
    }

    if (!validateReceivedFrame())
    {
        printTsv("CRC_ERR", 0, 0, 0, 0, _lastRxRssi, _lastRxSnr, 0, "bad_frame");
        startContinuousRx();
        return;
    }

    printFrameDebug("RX", _rxBuffer, (uint8_t)_rxSize);

    switch (_rxBuffer[0])
    {
    case FRAME_HELLO:
        processHelloFrame();
        break;
    case FRAME_ACK_CMD:
        processAckCmdFrame();
        break;
    default:
        printTsv("RX_UNKNOWN", _rxBuffer[1], _rxBuffer[2], 0, 0, _lastRxRssi, _lastRxSnr, _rxBuffer[0], "unsupported");
        startContinuousRx();
        break;
    }
}

void LoRaMaster::handleTxDone()
{
    if (_masterState == STATE_WAIT_ACK_HELLO_TXDONE)
    {
        printTsv("ACK_HELLO_TX_DONE", _currentHelloNodeId, _currentHelloSeq, 0, 0, 0, 0, 0, "ok");

        if (_pendingCmdAfterAck)
        {
            if (_pendingCmdIsQueued)
            {
                PendingNodeCommand *queued = getQueuedCommand(_pendingCmdTargetNode);
                if (queued != nullptr)
                {
                    queued->lastSeq = _pendingCmdSeq;
                    queued->retriesDone++;
                }
            }

            sendCmdNow(_pendingCmdTargetNode,
                       _pendingCmdSeq,
                       _pendingCmdCode,
                       _pendingCmdArgU16,
                       _pendingCmdText);

            _awaitingAckNodeId = _pendingCmdTargetNode;
            _awaitingAckSeq = _pendingCmdSeq;
            _awaitingAckCmd = _pendingCmdCode;
            _awaitingAckStartedMs = millis();

            clearPendingContext();
        }
        else
        {
            startContinuousRx();
        }
    }
    else if (_masterState == STATE_WAIT_CMD_TXDONE)
    {
        printTsv("CMD_TX_DONE", _txBuffer[1], _txBuffer[2], 0, 0, 0, 0, _txBuffer[4], "ok");
        startContinuousRx();
    }
    else
    {
        startContinuousRx();
    }
}

void LoRaMaster::handleTxTimeout()
{
    printTsv("TX_TIMEOUT", 0, 0, 0, 0, 0, 0, 0, "radio");

    clearPendingContext();

    if (_awaitingAckNodeId != 0)
    {
        PendingNodeCommand *queued = getQueuedCommand(_awaitingAckNodeId);
        if (queued != nullptr && queued->active)
        {
            publishQueuedCommandFailure(_awaitingAckNodeId, queued->cmdCode, queued->argU16);
            clearQueuedCommand(_awaitingAckNodeId);
        }
    }

    _awaitingAckNodeId = 0;
    _awaitingAckSeq = 0;
    _awaitingAckCmd = CMD_NONE;
    _awaitingAckStartedMs = 0;

    startContinuousRx();
}

void LoRaMaster::handleAckTimeouts()
{
    if (_awaitingAckNodeId == 0)
    {
        return;
    }

    if ((millis() - _awaitingAckStartedMs) < CMD_ACK_TIMEOUT_MS)
    {
        return;
    }

    uint8_t nodeId = _awaitingAckNodeId;
    uint8_t seq = _awaitingAckSeq;
    uint8_t cmd = _awaitingAckCmd;

    _awaitingAckNodeId = 0;
    _awaitingAckSeq = 0;
    _awaitingAckCmd = CMD_NONE;
    _awaitingAckStartedMs = 0;

    PendingNodeCommand *queued = getQueuedCommand(nodeId);
    if (queued != nullptr && queued->active && queued->awaitingAck && queued->lastSeq == seq && queued->cmdCode == cmd)
    {
        if (queued->retriesDone >= queued->maxRetries)
        {
            printTsv("CMD_DROP", nodeId, seq, 0, 0, 0, 0, cmd, "ack_timeout");
            publishQueuedCommandFailure(nodeId, queued->cmdCode, queued->argU16);
            clearQueuedCommand(nodeId);
        }
        else
        {
            printTsv("CMD_RETRY", nodeId, seq, 0, 0, 0, 0, cmd, "queued");
        }
    }

    startContinuousRx();
}

void LoRaMaster::publishPayload(const String &payload)
{
    if (_publishCallback != nullptr)
    {
        _publishCallback(payload);
    }
}

void LoRaMaster::publishNodePresence(uint8_t nodeId, bool online)
{
    String payload = "J;";
    payload += String(nodeId);
    payload += ";";
    payload += String(online ? 1 : 0);
    publishPayload(payload);
}

void LoRaMaster::publishNodeSeen(uint8_t nodeId)
{
    (void)nodeId;
}

void LoRaMaster::publishMasterListResult(bool ok)
{
    refreshOnlineStates();

    uint8_t count = 0;
    String ids;
    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (!_nodes[i].used || !_nodes[i].online)
        {
            continue;
        }
        if (count > 0)
        {
            ids += ",";
        }
        ids += String(_nodes[i].nodeId);
        count++;
    }

    String payload = "M;list;";
    payload += String(ok ? 1 : 0);
    payload += ";";
    payload += String(ok ? count : 0);
    payload += ";";
    payload += ok ? ids : String("");
    publishPayload(payload);
}

void LoRaMaster::publishMasterScheduleResult(bool ok, uint32_t seconds)
{
    String payload = "M;schedule;";
    payload += String(ok ? 1 : 0);
    payload += ";";
    payload += String(seconds);
    publishPayload(payload);
}

void LoRaMaster::publishPingResult(uint8_t nodeId, bool ok)
{
    String payload = "N;" + String(nodeId) + ";ping;" + String(ok ? 1 : 0);
    publishPayload(payload);
}

void LoRaMaster::publishBatteryResult(uint8_t nodeId, bool ok, uint16_t battery_mV, uint8_t statusFlag, int16_t rssi, int8_t snr)
{
    String payload = "N;" + String(nodeId) + ";bat;" + String(ok ? 1 : 0) + ";";
    if (ok)
    {
        payload += String(battery_mV);
        payload += ";";
        payload += String(statusFlag);
        payload += ";";
        payload += String(rssi);
        payload += ";";
        payload += String((int)snr);
    }
    else
    {
        payload += ";;;";
    }
    publishPayload(payload);
}

void LoRaMaster::publishStatusResult(uint8_t nodeId, bool ok, uint16_t battery_mV, uint8_t statusFlag, uint32_t uptimeSec, int16_t rssi, int8_t snr)
{
    String payload = "N;" + String(nodeId) + ";status;" + String(ok ? 1 : 0) + ";";
    if (ok)
    {
        payload += String(battery_mV);
        payload += ";";
        payload += String(statusFlag);
        payload += ";";
        payload += String((unsigned long)uptimeSec);
        payload += ";";
        payload += String(rssi);
        payload += ";";
        payload += String((int)snr);
    }
    else
    {
        payload += ";;;;";
    }
    publishPayload(payload);
}

void LoRaMaster::publishTextResult(uint8_t nodeId, bool ok)
{
    String payload = "N;" + String(nodeId) + ";text;" + String(ok ? 1 : 0);
    publishPayload(payload);
}

void LoRaMaster::publishRebootResult(uint8_t nodeId, bool ok)
{
    String payload = "N;" + String(nodeId) + ";reboot;" + String(ok ? 1 : 0);
    publishPayload(payload);
}

void LoRaMaster::publishPeriodResult(uint8_t nodeId, bool ok, uint16_t seconds)
{
    String payload = "N;" + String(nodeId) + ";period;" + String(ok ? 1 : 0) + ";" + String(seconds);
    publishPayload(payload);
}

void LoRaMaster::publishLedResult(uint8_t nodeId, bool ok, uint8_t value)
{
    String payload = "N;" + String(nodeId) + ";led;" + String(ok ? 1 : 0) + ";" + String(value);
    publishPayload(payload);
}

void LoRaMaster::publishValveResult(uint8_t nodeId, bool ok, uint8_t value)
{
    String payload = "N;" + String(nodeId) + ";valve;" + String(ok ? 1 : 0) + ";" + String(value);
    publishPayload(payload);
}

void LoRaMaster::publishWaterStartResult(uint8_t nodeId, bool ok,
                                         uint8_t valveOpen,
                                         uint8_t wateringActive,
                                         uint16_t durationSec,
                                         uint16_t remainingSec,
                                         uint32_t flowPulses,
                                         uint16_t litres,
                                         uint32_t postClosePulses,
                                         uint8_t postCloseLeakDetected)
{
    String payload = "N;";
    payload += String(nodeId);
    payload += ";water;";
    payload += String(ok ? 1 : 0);

    if (ok)
    {
        payload += ";";
        payload += String(valveOpen);
        payload += ";";
        payload += String(wateringActive);
        payload += ";";
        payload += String(durationSec);
        payload += ";";
        payload += String(remainingSec);
        payload += ";";
        payload += String((uint32_t)flowPulses);
        payload += ";";
        payload += String(litres);
        payload += ";";
        payload += String((uint32_t)postClosePulses);
        payload += ";";
        payload += String(postCloseLeakDetected);
    }

    publishPayload(payload);
}

void LoRaMaster::publishWaterAbortResult(uint8_t nodeId, bool ok,
                                         uint8_t valveOpen,
                                         uint8_t wateringActive,
                                         uint16_t durationSec,
                                         uint16_t remainingSec,
                                         uint32_t flowPulses,
                                         uint16_t litres,
                                         uint32_t postClosePulses,
                                         uint8_t postCloseLeakDetected)
{
    String payload = "N;";
    payload += String(nodeId);
    payload += ";abort;";
    payload += String(ok ? 1 : 0);

    if (ok)
    {
        payload += ";";
        payload += String(valveOpen);
        payload += ";";
        payload += String(wateringActive);
        payload += ";";
        payload += String(durationSec);
        payload += ";";
        payload += String(remainingSec);
        payload += ";";
        payload += String((uint32_t)flowPulses);
        payload += ";";
        payload += String(litres);
        payload += ";";
        payload += String((uint32_t)postClosePulses);
        payload += ";";
        payload += String(postCloseLeakDetected);
    }

    publishPayload(payload);
}

void LoRaMaster::publishWaterStatusResult(uint8_t nodeId, bool ok,
                                          uint8_t valveOpen,
                                          uint8_t wateringActive,
                                          uint16_t durationSec,
                                          uint16_t remainingSec,
                                          uint32_t flowPulses,
                                          uint16_t litres,
                                          uint32_t postClosePulses,
                                          uint8_t postCloseLeakDetected)
{
    String payload = "N;";
    payload += String(nodeId);
    payload += ";wstatus;";
    payload += String(ok ? 1 : 0);

    if (ok)
    {
        payload += ";";
        payload += String(valveOpen);
        payload += ";";
        payload += String(wateringActive);
        payload += ";";
        payload += String(durationSec);
        payload += ";";
        payload += String(remainingSec);
        payload += ";";
        payload += String((uint32_t)flowPulses);
        payload += ";";
        payload += String(litres);
        payload += ";";
        payload += String((uint32_t)postClosePulses);
        payload += ";";
        payload += String(postCloseLeakDetected);
    }

    publishPayload(payload);
}

void LoRaMaster::publishQueuedCommandFailure(uint8_t nodeId, uint8_t cmdCode, uint16_t argU16)
{
    switch (cmdCode)
    {
    case CMD_PING:
        publishPingResult(nodeId, false);
        break;
    case CMD_BATTERY:
        publishBatteryResult(nodeId, false, 0, 0, 0, 0);
        break;
    case CMD_STATUS:
        publishStatusResult(nodeId, false, 0, 0, 0, 0, 0);
        break;
    case CMD_TEXT_MESSAGE:
        publishTextResult(nodeId, false);
        break;
    case CMD_REBOOT:
        publishRebootResult(nodeId, false);
        break;
    case CMD_SET_PERIOD:
        publishPeriodResult(nodeId, false, argU16);
        break;
    case CMD_LED_ON:
        publishLedResult(nodeId, false, 1);
        break;
    case CMD_LED_OFF:
        publishLedResult(nodeId, false, 0);
        break;
    case CMD_VALVE_OPEN:
        publishValveResult(nodeId, false, 1);
        break;
    case CMD_VALVE_CLOSE:
        publishValveResult(nodeId, false, 0);
        break;

    case CMD_WATER_TIME:
        publishWaterStartResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    case CMD_WATER_ABORT:
        publishWaterAbortResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        break;

    case CMD_WATER_STATUS:
        publishWaterStatusResult(nodeId, false, 0, 0, 0, 0, 0, 0, 0, 0);
        break;
    case CMD_OTA_MODE:
        publishNodeOtaResult(nodeId, false);
        break;
    default:
        break;
    }
}

bool LoRaMaster::parseMqttMasterCommand(const String &payload, String &verb, String &argText) const
{
    verb = "";
    argText = "";

    int p1 = payload.indexOf(';');
    if (p1 < 0)
    {
        return false;
    }

    String head = payload.substring(0, p1);
    if (head != "M" && head != "m")
    {
        return false;
    }

    int p2 = payload.indexOf(';', p1 + 1);
    if (p2 < 0)
    {
        verb = payload.substring(p1 + 1);
        verb.trim();
        verb.toLowerCase();
        return true;
    }

    verb = payload.substring(p1 + 1, p2);
    verb.trim();
    verb.toLowerCase();

    argText = payload.substring(p2 + 1);
    argText.trim();
    return true;
}

bool LoRaMaster::parseMqttNodeCommand(const String &payload,
                                      uint8_t &nodeId,
                                      uint8_t &cmdCode,
                                      uint16_t &argU16,
                                      String &textArg,
                                      String &verb,
                                      String &argText) const
{
    nodeId = 0;
    cmdCode = CMD_NONE;
    argU16 = 0;
    textArg = "";
    verb = "";
    argText = "";

    int p1 = payload.indexOf(';');
    if (p1 < 0)
    {
        return false;
    }

    String head = payload.substring(0, p1);
    if (head != "N" && head != "n")
    {
        return false;
    }

    int p2 = payload.indexOf(';', p1 + 1);
    if (p2 < 0)
    {
        return false;
    }

    int p3 = payload.indexOf(';', p2 + 1);

    nodeId = (uint8_t)payload.substring(p1 + 1, p2).toInt();
    verb = payload.substring(p2 + 1, (p3 >= 0) ? p3 : payload.length());
    verb.trim();
    verb.toLowerCase();

    if (p3 >= 0)
    {
        argText = payload.substring(p3 + 1);
        argText.trim();
    }

    if (verb == "bat" || verb == "battery")
    {
        cmdCode = CMD_BATTERY;
        verb = "bat";
        return true;
    }
    if (verb == "status")
    {
        cmdCode = CMD_STATUS;
        return true;
    }
    if (verb == "wstatus")
    {
        cmdCode = CMD_WATER_STATUS;
        return true;
    }
    if (verb == "reboot")
    {
        cmdCode = CMD_REBOOT;
        return true;
    }
    if (verb == "ping")
    {
        cmdCode = CMD_PING;
        return true;
    }
    if (verb == "abort")
    {
        cmdCode = CMD_WATER_ABORT;
        return true;
    }
    if (verb == "text")
    {
        cmdCode = CMD_TEXT_MESSAGE;
        textArg = argText;
        return true;
    }
    if (verb == "led")
    {
        if (argText == "1")
        {
            cmdCode = CMD_LED_ON;
            return true;
        }
        if (argText == "0")
        {
            cmdCode = CMD_LED_OFF;
            return true;
        }
        return false;
    }
    if (verb == "period")
    {
        uint32_t sec = (uint32_t)argText.toInt();
        if (sec == 0 || sec > 65535UL)
        {
            cmdCode = CMD_SET_PERIOD;
            argU16 = (uint16_t)sec;
            return false;
        }
        cmdCode = CMD_SET_PERIOD;
        argU16 = (uint16_t)sec;
        return true;
    }
    if (verb == "water")
    {
        uint32_t sec = (uint32_t)argText.toInt();
        cmdCode = CMD_WATER_TIME;
        argU16 = (uint16_t)sec;
        if (sec == 0 || sec > 65535UL)
        {
            return false;
        }
        return true;
    }
    if (verb == "valve")
    {
        if (argText == "1")
        {
            cmdCode = CMD_VALVE_OPEN;
            return true;
        }
        if (argText == "0")
        {
            cmdCode = CMD_VALVE_CLOSE;
            return true;
        }
        return false;
    }
    if (verb == "open")
    {
        cmdCode = CMD_VALVE_OPEN;
        argText = "1";
        verb = "valve";
        return true;
    }
    if (verb == "close")
    {
        cmdCode = CMD_VALVE_CLOSE;
        argText = "0";
        verb = "valve";
        return true;
    }
    if (verb == "ota")
    {
        cmdCode = CMD_OTA_MODE;
        return true;
    }

    return false;
}

void LoRaMaster::onTxDone()
{
    if (_instance != nullptr)
    {
        _instance->_txDoneFlag = true;
    }
}

void LoRaMaster::onTxTimeout()
{
    if (_instance != nullptr)
    {
        _instance->_txTimeoutFlag = true;
    }
}

void LoRaMaster::onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (_instance == nullptr)
    {
        return;
    }

    if (size > MAX_BUFFER_SIZE)
    {
        size = MAX_BUFFER_SIZE;
    }

    memcpy(_instance->_rxBuffer, payload, size);
    _instance->_rxSize = size;
    _instance->_lastRxRssi = rssi;
    _instance->_lastRxSnr = snr;
    _instance->_rxDoneFlag = true;
}

String LoRaMaster::getOnlineNodeListCsv() const
{
    String out;
    bool first = true;

    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (!_nodes[i].used || !_nodes[i].online)
        {
            continue;
        }

        if (!first)
        {
            out += ",";
        }

        out += String(_nodes[i].nodeId);
        first = false;
    }

    return out;
}

void LoRaMaster::publishMasterOtaResult(bool ok, uint32_t seconds)
{
    String payload = "M;ota;";
    payload += String(ok ? 1 : 0);
    payload += ";";
    payload += String(seconds);
    publishPayload(payload);
}

void LoRaMaster::publishNodeOtaResult(uint8_t nodeId, bool ok)
{
    String payload = "N;" + String(nodeId) + ";ota;" + String(ok ? 1 : 0);
    publishPayload(payload);
}

void LoRaMaster::publishInvalidNodeCommandResult(const String &rawPayload)
{
    String payload = "N;0;invalid;0;";
    payload += rawPayload;
    publishPayload(payload);
}

void LoRaMaster::publishNodeBootResult(uint8_t nodeId)
{
    String payload = "N;";
    payload += String(nodeId);
    payload += ";boot;1";
    publishPayload(payload);
}

void LoRaMaster::republishAllNodePresence()
{
    refreshOnlineStates();
    for (uint8_t i = 0; i < MAX_NODES; i++)
    {
        if (_nodes[i].used)
        {
            publishNodePresence(_nodes[i].nodeId, _nodes[i].online);
        }
    }
}

void LoRaMaster::setMasterBatteryReader(MasterBatteryReader reader)
{
    _masterBatteryReader = reader;
}

void LoRaMaster::setMasterRebootHandler(MasterRebootHandler handler)
{
    _masterRebootHandler = handler;
}
