/**
 * @file lora_master.h
 * @brief LoRa P2P master controller interface.
 *
 * This file declares the LoRaMaster class, which manages the radio master side
 * of the watering network. It receives node HELLO frames, sends acknowledgments
 * and commands, tracks node presence and status, and publishes command results
 * through a callback typically backed by MQTT.
 */
#pragma once

#include <Arduino.h>

/**
 * @class LoRaMaster
 * @brief Master-side LoRa P2P protocol controller.
 *
 * LoRaMaster owns the SX126x radio event flow, node table, pending command
 * queue, scheduled battery/status polling, and result publishing helpers. The
 * class is intended to be driven from the Arduino loop() function by calling
 * begin() once and loop() repeatedly.
 *
 * @warning The class keeps internal fixed-size buffers and node tables. Inputs
 * that become frame payloads or queued text are truncated or rejected according
 * to the implementation limits.
 */
class LoRaMaster
{
public:
    /**
     * @brief Callback type used to publish serialized result payloads.
     *
     * @param[in] payload Serialized payload to publish.
     * @return true if the payload was accepted by the publisher, false otherwise.
     */
    using PublishCallback = bool (*)(const String &payload);

    /**
     * @brief Constructs a LoRaMaster instance with default internal state.
     */
    LoRaMaster();

    /**
     * @brief Callback type used to read the master battery voltage.
     *
     * @return Master battery voltage in millivolts.
     */
    typedef uint16_t (*MasterBatteryReader)();
    /**
     * @brief Callback type used to request a master reboot.
     */
    typedef void (*MasterRebootHandler)();

    /**
     * @brief Initializes the radio and prints the initial diagnostic header.
     *
     * @post The radio is configured and continuous receive mode is started.
     */
    void begin();
    /**
     * @brief Runs one LoRa master service iteration.
     *
     * The loop refreshes online states, starts schedule cycles when needed,
     * handles pending ACK timeouts, and processes radio events.
     */
    void loop();

    /**
     * @brief Registers the function used to read the master battery.
     *
     * @param[in] reader Function pointer to call when a master battery value is needed.
     */
    void setMasterBatteryReader(MasterBatteryReader reader);
    /**
     * @brief Registers the function used to reboot the master.
     *
     * @param[in] handler Function pointer invoked for master reboot commands.
     */
    void setMasterRebootHandler(MasterRebootHandler handler);

    /**
     * @brief Handles one command line from the serial console or MQTT bridge.
     *
     * @param[in] line Command line text to parse.
     */
    void handleConsoleLine(const String &line);
    /**
     * @brief Parses and handles one MQTT command payload.
     *
     * @param[in] payload Raw MQTT command payload.
     * @return true if the payload was recognized as a supported command, false otherwise.
     */
    bool handleMqttCommand(const String &payload);

    /**
     * @brief Sets the publishing callback used by result helpers.
     *
     * @param[in] cb Callback to call for outgoing payloads, or nullptr to disable publishing.
     */
    void setPublishCallback(PublishCallback cb);

    /**
     * @brief Returns the number of nodes currently considered online.
     *
     * @return Count of online nodes in the internal node table.
     */
    uint8_t getOnlineNodeCount() const;
    /**
     * @brief Sets the automatic battery schedule period.
     *
     * @param[in] seconds Period in seconds.
     */
    void setBatterySchedulePeriodSec(uint32_t seconds);
    /**
     * @brief Builds a CSV list of currently online node identifiers.
     *
     * @return CSV string containing online node IDs.
     */
    String getOnlineNodeListCsv() const;
    /**
     * @brief Publishes the OTA result for a node.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     */
    void publishNodeOtaResult(uint8_t nodeId, bool ok);
    /**
     * @brief Republishes presence information for all known nodes.
     */
    void republishAllNodePresence();

private:
    /**
     * @brief Callback used to read the master battery voltage.
     */
    MasterBatteryReader _masterBatteryReader;
    /**
     * @brief Callback used to reboot the master.
     */
    MasterRebootHandler _masterRebootHandler;
    /**
     * @enum FrameType
     * @brief Frame types exchanged by the master and nodes.
     */
    enum FrameType : uint8_t
    {
        FRAME_HELLO = 0x10,     ///< Node announcement frame.
        FRAME_ACK_HELLO = 0x11, ///< Master acknowledgment for a HELLO frame.
        FRAME_CMD = 0x20,       ///< Command frame sent by the master.
        FRAME_ACK_CMD = 0x21    ///< Node acknowledgment for a command frame.
    };

    /**
     * @enum CommandType
     * @brief Application command codes transported in FRAME_CMD and FRAME_ACK_CMD payloads.
     */
    enum CommandType : uint8_t
    {
        CMD_NONE = 0x00,         ///< No command.
        CMD_PING = 0x01,         ///< Connectivity test command.
        CMD_LED_ON = 0x02,       ///< Request to switch the node LED on.
        CMD_LED_OFF = 0x03,      ///< Request to switch the node LED off.
        CMD_TEXT_MESSAGE = 0x10, ///< Text message command.
        CMD_BATTERY = 0x11,      ///< Battery measurement command.
        CMD_REBOOT = 0x12,       ///< Node reboot command.
        CMD_STATUS = 0x13,       ///< Node status query command.
        CMD_SET_PERIOD = 0x14,   ///< Node period configuration command.
        CMD_VALVE_OPEN = 0x20,   ///< Watering valve open command.
        CMD_VALVE_CLOSE = 0x21,  ///< Watering valve close command.
        CMD_WATER_TIME = 0x22,   ///< Timed watering start command.
        CMD_WATER_ABORT = 0x23,  ///< Watering abort command.
        CMD_WATER_STATUS = 0x24, ///< Watering status query command.
        CMD_OTA_MODE = 0x30      ///< Node OTA mode command.
    };

    /**
     * @enum AckStatus
     * @brief Status byte values returned by nodes in command acknowledgments.
     */
    enum AckStatus : uint8_t
    {
        ACK_STATUS_OK = 0x00,   ///< Command accepted or completed.
        ACK_STATUS_ERROR = 0x01 ///< Command rejected or failed.
    };

    /**
     * @enum MasterState
     * @brief Radio state machine states used by the master.
     */
    enum MasterState : uint8_t
    {
        STATE_RX = 0,                ///< Continuous receive mode.
        STATE_WAIT_ACK_HELLO_TXDONE, ///< Waiting for HELLO acknowledgment TX completion.
        STATE_WAIT_CMD_TXDONE        ///< Waiting for command TX completion.
    };

    /**
     * @struct NodeInfo
     * @brief Runtime state cached for one known LoRa node.
     */
    struct NodeInfo
    {
        bool used;                      ///< true when this table entry is allocated.
        bool online;                    ///< true when the node is currently considered online.
        bool notifiedOnline;            ///< true after an online notification was published.
        bool notifiedLost;              ///< true after a lost notification was published.
        uint8_t nodeId;                 ///< Node identifier.
        uint8_t lastHelloSeq;           ///< Last HELLO sequence number received from the node.
        uint8_t statusFlag;             ///< Last node status flag.
        uint16_t lastBattery_mV;        ///< Last known battery voltage, in millivolts.
        int16_t lastRssi;               ///< Last received RSSI value.
        int8_t lastSnr;                 ///< Last received SNR value.
        uint32_t lastSeenMs;            ///< millis() timestamp of the last node activity.
        uint32_t lastCycleIdHandled;    ///< Last schedule cycle handled for this node.
        uint8_t lastValveOpen;          ///< Last reported valve-open state.
        uint8_t lastWateringActive;     ///< Last reported watering-active state.
        uint16_t lastWaterDurationSec;  ///< Last reported watering duration, in seconds.
        uint16_t lastWaterRemainingSec; ///< Last reported remaining watering time, in seconds.
        uint32_t lastFlowPulses;        ///< Last reported flow sensor pulse count.
        uint16_t lastLitres;            ///< Last reported water volume, in litres as encoded by the node.
    };

    /**
     * @struct PendingNodeCommand
     * @brief Queued or in-flight command targeting one node.
     */
    struct PendingNodeCommand
    {
        bool active;          ///< true when this command slot is in use.
        bool awaitingAck;     ///< true while waiting for a node ACK.
        uint8_t targetNode;   ///< Target node identifier.
        uint8_t cmdCode;      ///< Command code to send.
        uint16_t argU16;      ///< Optional 16-bit command argument.
        char text[48];        ///< Optional text payload buffer.
        char mqttVerb[16];    ///< Original MQTT verb associated with the command.
        char mqttArgText[24]; ///< Original MQTT argument text associated with the command.
        uint8_t lastSeq;      ///< Last sequence number used for this command.
        uint8_t retriesDone;  ///< Number of retries already attempted.
        uint8_t maxRetries;   ///< Maximum number of retries allowed.
    };

    /**
     * @struct ScheduleState
     * @brief State of the periodic node polling schedule.
     */
    struct ScheduleState
    {
        bool enabled;       ///< true when automatic scheduling is enabled.
        bool cycleActive;   ///< true while a schedule cycle is in progress.
        uint32_t periodMs;  ///< Schedule period in milliseconds.
        uint32_t nextDueMs; ///< millis() timestamp for the next cycle start.
        uint32_t cycleId;   ///< Monotonic schedule cycle identifier.
    };

    static constexpr uint32_t RF_FREQUENCY = 865000000UL;  ///< LoRa RF frequency, in hertz.
    static constexpr int8_t TX_OUTPUT_POWER = 14;          ///< LoRa transmit power, in dBm.
    static constexpr uint8_t LORA_BANDWIDTH = 0;           ///< SX126x bandwidth code; 0 represents 125 kHz in this configuration.
    static constexpr uint8_t LORA_SPREADING_FACTOR = 7;    ///< LoRa spreading factor.
    static constexpr uint8_t LORA_CODINGRATE = 1;          ///< SX126x coding rate code.
    static constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;    ///< LoRa preamble length, in symbols.
    static constexpr uint16_t LORA_SYMBOL_TIMEOUT = 0;     ///< LoRa symbol timeout for receive configuration.
    static constexpr bool LORA_FIX_LENGTH_PAYLOAD = false; ///< false to use variable-length packets.
    static constexpr bool LORA_IQ_INVERSION = false;       ///< false to keep standard IQ polarity.

    static constexpr uint32_t TX_TIMEOUT_VALUE = 3000UL;                          ///< Radio TX timeout, in milliseconds.
    static constexpr uint32_t CMD_ACK_TIMEOUT_MS = 1600UL;                        ///< Command ACK wait timeout, in milliseconds.
    static constexpr uint8_t MAX_BUFFER_SIZE = 96;                                ///< Maximum radio frame buffer size, in bytes.
    static constexpr uint8_t MAX_NODES = 16;                                      ///< Maximum number of node records tracked by the master.
    static constexpr uint32_t NODE_ONLINE_TIMEOUT_MS = 120000UL;                  ///< Inactivity delay before a node is considered offline.
    static constexpr uint32_t DEFAULT_BATTERY_SCHEDULE_MS = 10UL * 60UL * 1000UL; ///< Default periodic schedule interval.
    static constexpr uint8_t MAX_CMD_RETRIES = 3;                                 ///< Default maximum retry count for queued node commands.

    PublishCallback _publishCallback; ///< Callback used to publish serialized events and results.

    volatile bool _rxDoneFlag;    ///< Set by the RX callback when a frame is received.
    volatile bool _txDoneFlag;    ///< Set by the TX callback when an emission completes.
    volatile bool _txTimeoutFlag; ///< Set by the TX timeout callback.
    volatile uint16_t _rxSize;    ///< Size of the last received frame in _rxBuffer.
    volatile int16_t _lastRxRssi; ///< RSSI value associated with the last received frame.
    volatile int8_t _lastRxSnr;   ///< SNR value associated with the last received frame.

    MasterState _masterState; ///< Current master radio state.

    uint8_t _rxBuffer[MAX_BUFFER_SIZE]; ///< Receive buffer filled by the radio callback.
    uint8_t _txBuffer[MAX_BUFFER_SIZE]; ///< Transmit buffer used by frame builders.

    uint8_t _currentHelloNodeId; ///< Node ID associated with the HELLO currently being acknowledged.
    uint8_t _currentHelloSeq;    ///< Sequence number associated with the HELLO currently being acknowledged.

    bool _pendingCmdAfterAck;      ///< true when a command must be sent after a HELLO ACK completes.
    uint8_t _pendingCmdTargetNode; ///< Target node for the pending command.
    uint8_t _pendingCmdSeq;        ///< Sequence number for the pending command.
    uint8_t _pendingCmdCode;       ///< Command code for the pending command.
    uint16_t _pendingCmdArgU16;    ///< Optional 16-bit argument for the pending command.
    char _pendingCmdText[48];      ///< Optional text payload for the pending command.
    bool _pendingCmdIsQueued;      ///< true when the pending command originated from the queue.
    bool _pendingCmdIsScheduled;   ///< true when the pending command originated from the scheduler.

    uint8_t _awaitingAckNodeId;     ///< Node ID for the command currently awaiting ACK.
    uint8_t _awaitingAckSeq;        ///< Sequence number for the command currently awaiting ACK.
    uint8_t _awaitingAckCmd;        ///< Command code currently awaiting ACK.
    uint32_t _awaitingAckStartedMs; ///< millis() timestamp when ACK waiting started.

    NodeInfo _nodes[MAX_NODES];                    ///< Fixed-size node state table.
    PendingNodeCommand _queuedCmds[MAX_NODES + 1]; ///< Fixed-size command queue indexed by node slot.
    ScheduleState _schedule;                       ///< Periodic schedule runtime state.

    static LoRaMaster *_instance; ///< Singleton-like pointer used by static radio callbacks.

private:
    /**
     * @brief Configures the radio driver and callback table.
     */
    void initRadio();
    /**
     * @brief Clears the radio event flags and received size.
     */
    void resetFlags();
    /**
     * @brief Starts continuous radio reception.
     */
    void startContinuousRx();

    /**
     * @brief Computes a CRC-16/CCITT checksum.
     *
     * @param[in] data Pointer to the bytes to checksum.
     * @param[in] len Number of bytes to process.
     * @return Computed CRC value.
     */
    uint16_t crc16Ccitt(const uint8_t *data, uint16_t len) const;
    /**
     * @brief Returns the total frame length for a payload length.
     *
     * @param[in] payloadLen Payload length in bytes.
     * @return Total encoded frame length in bytes.
     */
    uint8_t getFrameTotalLength(uint8_t payloadLen) const;
    /**
     * @brief Builds a protocol frame in the transmit buffer.
     *
     * @param[in] type Frame type.
     * @param[in] nodeId Node identifier.
     * @param[in] seq Sequence number.
     * @param[in] payload Payload source pointer.
     * @param[in] len Payload length in bytes.
     * @return Total frame length written to _txBuffer.
     * @warning payload must be valid when len is non-zero.
     */
    uint8_t buildFrame(uint8_t type, uint8_t nodeId, uint8_t seq, const uint8_t *payload, uint8_t len);
    /**
     * @brief Validates the last frame stored in the receive buffer.
     *
     * @return true if the received frame passes length and checksum checks, false otherwise.
     */
    bool validateReceivedFrame() const;
    /**
     * @brief Prints a frame in diagnostic form.
     *
     * @param[in] prefix Prefix string printed before frame bytes.
     * @param[in] buf Buffer containing the frame bytes.
     * @param[in] len Number of bytes to print.
     */
    void printFrameDebug(const char *prefix, const uint8_t *buf, uint8_t len) const;

    /**
     * @brief Prints the TSV diagnostic header.
     */
    void printTsvHeader();
    /**
     * @brief Prints one TSV diagnostic event row.
     *
     * @param[in] eventName Event label.
     * @param[in] nodeId Node identifier.
     * @param[in] seq Frame sequence number.
     * @param[in] battery_mV Battery voltage in millivolts.
     * @param[in] statusFlag Node status flag.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     * @param[in] cmd Command code associated with the event.
     * @param[in] info Additional textual information.
     */
    void printTsv(const char *eventName,
                  uint8_t nodeId,
                  uint8_t seq,
                  uint16_t battery_mV,
                  uint8_t statusFlag,
                  int16_t rssi,
                  int8_t snr,
                  uint8_t cmd,
                  const char *info);

    /**
     * @brief Finds the node table index for a node ID.
     *
     * @param[in] nodeId Node identifier to search.
     * @return Node table index, or -1 if not found.
     */
    int findNodeIndex(uint8_t nodeId) const;
    /**
     * @brief Allocates or reuses a node table entry for a node ID.
     *
     * @param[in] nodeId Node identifier to allocate.
     * @return Node table index, or -1 if no slot is available.
     */
    int allocateNodeIndex(uint8_t nodeId);
    /**
     * @brief Updates node state from a received HELLO frame.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] seq HELLO sequence number.
     * @param[in] statusFlag Node status flag.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     */
    void updateNodeHello(uint8_t nodeId, uint8_t seq, uint8_t statusFlag, int16_t rssi, int8_t snr);
    /**
     * @brief Updates the cached battery voltage for a node.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] battery_mV Battery voltage in millivolts.
     */
    void updateNodeBattery(uint8_t nodeId, uint16_t battery_mV);
    /**
     * @brief Updates the cached general status for a node.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] battery_mV Battery voltage in millivolts.
     * @param[in] statusFlag Node status flag.
     * @param[in] uptimeSec Node uptime in seconds.
     * @param[in] rssiAbs Absolute RSSI value as encoded by the node.
     * @param[in] snr SNR value.
     */
    void updateNodeStatus(uint8_t nodeId,
                          uint16_t battery_mV,
                          uint8_t statusFlag,
                          uint32_t uptimeSec,
                          uint16_t rssiAbs,
                          int8_t snr);
    /**
     * @brief Updates the cached watering state for a node.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] valveOpen Valve state reported by the node.
     * @param[in] wateringActive Watering active state reported by the node.
     * @param[in] waterDurationSec Configured or reported watering duration, in seconds.
     * @param[in] remainingSec Remaining watering time, in seconds.
     * @param[in] flowPulses Flow sensor pulse count.
     * @param[in] litres Water volume reported by the node.
     */
    void updateNodeWaterStatus(uint8_t nodeId,
                               uint8_t valveOpen,
                               uint8_t wateringActive,
                               uint16_t waterDurationSec,
                               uint16_t remainingSec,
                               uint32_t flowPulses,
                               uint16_t litres);

    /**
     * @brief Refreshes online/offline states from node timestamps.
     */
    void refreshOnlineStates();
    /**
     * @brief Dumps the known node list as TSV diagnostics.
     */
    void dumpNodeListTsv();

    /**
     * @brief Returns the mutable queued command slot for a node.
     *
     * @param[in] nodeId Node identifier.
     * @return Pointer to the queued command slot, or nullptr if unavailable.
     */
    PendingNodeCommand *getQueuedCommand(uint8_t nodeId);
    /**
     * @brief Returns the queued command slot for a node.
     *
     * @param[in] nodeId Node identifier.
     * @return Pointer to the queued command slot, or nullptr if unavailable.
     */
    const PendingNodeCommand *getQueuedCommand(uint8_t nodeId) const;
    /**
     * @brief Checks whether a node has an active queued command.
     *
     * @param[in] nodeId Node identifier.
     * @return true if a queued command exists for the node, false otherwise.
     */
    bool hasQueuedCommand(uint8_t nodeId) const;
    /**
     * @brief Clears the queued command slot for a node.
     *
     * @param[in] nodeId Node identifier.
     */
    void clearQueuedCommand(uint8_t nodeId);
    /**
     * @brief Queues a command for later transmission to a node.
     *
     * @param[in] nodeId Target node identifier.
     * @param[in] cmdCode Command code.
     * @param[in] argU16 Optional 16-bit argument.
     * @param[in] text Optional text payload.
     * @param[in] mqttVerb MQTT verb associated with the command.
     * @param[in] mqttArgText MQTT argument text associated with the command.
     * @return true if the command was queued, false otherwise.
     */
    bool queueNodeCommand(uint8_t nodeId,
                          uint8_t cmdCode,
                          uint16_t argU16,
                          const char *text,
                          const char *mqttVerb,
                          const char *mqttArgText);

    /**
     * @brief Clears the pending command context used between HELLO ACK and command TX.
     */
    void clearPendingContext();
    /**
     * @brief Starts a periodic schedule cycle when it becomes due.
     */
    void maybeStartScheduleCycle();
    /**
     * @brief Checks whether a node still has work in the current schedule cycle.
     *
     * @param[in] nodeId Node identifier.
     * @return true if the node should still be polled in the current cycle, false otherwise.
     */
    bool hasScheduleWorkForNode(uint8_t nodeId) const;
    /**
     * @brief Marks the scheduled work for a node as complete.
     *
     * @param[in] nodeId Node identifier.
     */
    void markScheduleDoneForNode(uint8_t nodeId);
    /**
     * @brief Checks whether the active schedule cycle is complete.
     *
     * @return true if no remaining online node requires schedule work, false otherwise.
     */
    bool isScheduleCycleComplete() const;

    /**
     * @brief Sends the current transmit buffer through the radio driver.
     *
     * @param[in] totalLen Number of bytes to send from _txBuffer.
     * @return true if the driver accepted the frame for transmission, false otherwise.
     */
    bool sendFrame(uint8_t totalLen);
    /**
     * @brief Sends a HELLO acknowledgment to a node.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] seq Sequence number to acknowledge.
     */
    void sendAckHello(uint8_t nodeId, uint8_t seq);
    /**
     * @brief Sends a command frame immediately.
     *
     * @param[in] nodeId Target node identifier.
     * @param[in] seq Sequence number to use.
     * @param[in] cmdCode Command code.
     * @param[in] argU16 Optional 16-bit argument.
     * @param[in] textPayload Optional text payload.
     */
    void sendCmdNow(uint8_t nodeId, uint8_t seq, uint8_t cmdCode, uint16_t argU16, const char *textPayload);

    /**
     * @brief Dispatches the currently received frame by type.
     */
    void processReceivedFrame();
    /**
     * @brief Processes a received HELLO frame.
     */
    void processHelloFrame();
    /**
     * @brief Processes a received command acknowledgment frame.
     */
    void processAckCmdFrame();
    /**
     * @brief Handles a radio TX done event.
     */
    void handleTxDone();
    /**
     * @brief Handles a radio TX timeout event.
     */
    void handleTxTimeout();
    /**
     * @brief Handles command acknowledgment timeouts and retries.
     */
    void handleAckTimeouts();

    /**
     * @brief Publishes a serialized payload through _publishCallback.
     *
     * @param[in] payload Payload to publish.
     */
    void publishPayload(const String &payload);

    /**
     * @brief Publishes a node presence change.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] online true when the node is online, false when lost.
     */
    void publishNodePresence(uint8_t nodeId, bool online);
    /**
     * @brief Publishes a node-seen notification.
     *
     * @param[in] nodeId Node identifier.
     */
    void publishNodeSeen(uint8_t nodeId);

    /**
     * @brief Publishes the result of a master list command.
     *
     * @param[in] ok true if the command succeeded, false otherwise.
     */
    void publishMasterListResult(bool ok);
    /**
     * @brief Publishes the result of a master schedule command.
     *
     * @param[in] ok true if the schedule command succeeded, false otherwise.
     * @param[in] seconds Schedule period in seconds.
     */
    void publishMasterScheduleResult(bool ok, uint32_t seconds);

    /**
     * @brief Publishes the result of a master OTA command.
     *
     * @param[in] ok true if the command succeeded, false otherwise.
     * @param[in] seconds OTA duration in seconds.
     */
    void publishMasterOtaResult(bool ok, uint32_t seconds);

    /**
     * @brief Publishes the result of a ping command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     */
    void publishPingResult(uint8_t nodeId, bool ok);
    /**
     * @brief Publishes the result of a battery command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] battery_mV Battery voltage in millivolts.
     * @param[in] statusFlag Node status flag.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     */
    void publishBatteryResult(uint8_t nodeId, bool ok, uint16_t battery_mV, uint8_t statusFlag, int16_t rssi, int8_t snr);
    /**
     * @brief Publishes the result of a status command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] battery_mV Battery voltage in millivolts.
     * @param[in] statusFlag Node status flag.
     * @param[in] uptimeSec Node uptime in seconds.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     */
    void publishStatusResult(uint8_t nodeId, bool ok, uint16_t battery_mV, uint8_t statusFlag, uint32_t uptimeSec, int16_t rssi, int8_t snr);
    /**
     * @brief Publishes the result of a text command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     */
    void publishTextResult(uint8_t nodeId, bool ok);
    /**
     * @brief Publishes the result of a reboot command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     */
    void publishRebootResult(uint8_t nodeId, bool ok);
    /**
     * @brief Publishes the result of a period configuration command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] seconds Period in seconds.
     */
    void publishPeriodResult(uint8_t nodeId, bool ok, uint16_t seconds);
    /**
     * @brief Publishes the result of an LED command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] value LED command value.
     */
    void publishLedResult(uint8_t nodeId, bool ok, uint8_t value);
    /**
     * @brief Publishes the result of a valve command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] value Valve command value.
     */
    void publishValveResult(uint8_t nodeId, bool ok, uint8_t value);

    /**
     * @brief Publishes the result of a timed watering start command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] valveOpen Reported valve state.
     * @param[in] wateringActive Reported watering-active state.
     * @param[in] durationSec Watering duration in seconds.
     * @param[in] remainingSec Remaining watering time in seconds.
     * @param[in] flowPulses Flow sensor pulse count.
     * @param[in] litres Water volume reported by the node.
     * @param[in] postClosePulses Pulse count observed after valve close.
     * @param[in] postCloseLeakDetected Leak flag reported after valve close.
     */
    void publishWaterStartResult(uint8_t nodeId, bool ok,
                                 uint8_t valveOpen,
                                 uint8_t wateringActive,
                                 uint16_t durationSec,
                                 uint16_t remainingSec,
                                 uint32_t flowPulses,
                                 uint16_t litres,
                                 uint32_t postClosePulses,
                                 uint8_t postCloseLeakDetected);

    /**
     * @brief Publishes the result of a watering abort command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] valveOpen Reported valve state.
     * @param[in] wateringActive Reported watering-active state.
     * @param[in] durationSec Watering duration in seconds.
     * @param[in] remainingSec Remaining watering time in seconds.
     * @param[in] flowPulses Flow sensor pulse count.
     * @param[in] litres Water volume reported by the node.
     * @param[in] postClosePulses Pulse count observed after valve close.
     * @param[in] postCloseLeakDetected Leak flag reported after valve close.
     */
    void publishWaterAbortResult(uint8_t nodeId, bool ok,
                                 uint8_t valveOpen,
                                 uint8_t wateringActive,
                                 uint16_t durationSec,
                                 uint16_t remainingSec,
                                 uint32_t flowPulses,
                                 uint16_t litres,
                                 uint32_t postClosePulses,
                                 uint8_t postCloseLeakDetected);

    /**
     * @brief Publishes the result of a watering status command.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] ok true for success, false for failure.
     * @param[in] valveOpen Reported valve state.
     * @param[in] wateringActive Reported watering-active state.
     * @param[in] durationSec Watering duration in seconds.
     * @param[in] remainingSec Remaining watering time in seconds.
     * @param[in] flowPulses Flow sensor pulse count.
     * @param[in] litres Water volume reported by the node.
     * @param[in] postClosePulses Pulse count observed after valve close.
     * @param[in] postCloseLeakDetected Leak flag reported after valve close.
     */
    void publishWaterStatusResult(uint8_t nodeId, bool ok,
                                  uint8_t valveOpen,
                                  uint8_t wateringActive,
                                  uint16_t durationSec,
                                  uint16_t remainingSec,
                                  uint32_t flowPulses,
                                  uint16_t litres,
                                  uint32_t postClosePulses,
                                  uint8_t postCloseLeakDetected);

    /**
     * @brief Publishes a failure for a command that could not be delivered.
     *
     * @param[in] nodeId Node identifier.
     * @param[in] cmdCode Command code.
     * @param[in] argU16 Optional 16-bit argument.
     */
    void publishQueuedCommandFailure(uint8_t nodeId, uint8_t cmdCode, uint16_t argU16);
    /**
     * @brief Publishes a failure for an invalid node command payload.
     *
     * @param[in] rawPayload Original payload that could not be parsed.
     */
    void publishInvalidNodeCommandResult(const String &rawPayload);

    /**
     * @brief Parses an MQTT payload targeting the master itself.
     *
     * @param[in] payload Raw MQTT payload.
     * @param[out] verb Parsed command verb.
     * @param[out] argText Parsed argument text.
     * @return true if the payload is a valid master command, false otherwise.
     */
    bool parseMqttMasterCommand(const String &payload, String &verb, String &argText) const;
    /**
     * @brief Parses an MQTT payload targeting a LoRa node.
     *
     * @param[in] payload Raw MQTT payload.
     * @param[out] nodeId Parsed target node identifier.
     * @param[out] cmdCode Parsed command code.
     * @param[out] argU16 Parsed 16-bit argument.
     * @param[out] textArg Parsed text argument.
     * @param[out] verb Parsed command verb.
     * @param[out] argText Parsed raw argument text.
     * @return true if the payload is a valid node command, false otherwise.
     */
    bool parseMqttNodeCommand(const String &payload,
                              uint8_t &nodeId,
                              uint8_t &cmdCode,
                              uint16_t &argU16,
                              String &textArg,
                              String &verb,
                              String &argText) const;

    /**
     * @brief Publishes a node boot notification.
     *
     * @param[in] nodeId Node identifier.
     */
    void publishNodeBootResult(uint8_t nodeId);
    /**
     * @brief Static radio TX done callback bridge.
     */
    static void onTxDone();
    /**
     * @brief Static radio TX timeout callback bridge.
     */
    static void onTxTimeout();
    /**
     * @brief Static radio RX done callback bridge.
     *
     * @param[in] payload Received payload pointer provided by the radio driver.
     * @param[in] size Number of received bytes.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     */
    static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
};
