/**
 * @file bg77_mqtt.h
 * @brief MQTT client wrapper for the Quectel BG77 LTE modem.
 *
 * This file declares the Bg77Mqtt class, which drives the BG77 modem through
 * AT commands, maintains LTE packet and MQTT state, receives subscribed MQTT
 * messages, and publishes payloads to the configured MQTT topics.
 */
#pragma once

#include <Arduino.h>

/**
 * @class Bg77Mqtt
 * @brief BG77 LTE/MQTT state machine.
 *
 * The class owns modem serial I/O, BG77 power-key control, network attachment,
 * MQTT connection management, a small inbound MQTT queue, and console command
 * handling. The owner is expected to call begin() once and loop() repeatedly.
 */
class Bg77Mqtt
{
public:
    /**
     * @brief Constructs a BG77 MQTT driver.
     *
     * @param[in,out] pcSerial Stream used for diagnostics.
     * @param[in,out] modemSerial UART connected to the BG77 modem.
     * @param[in] pwrKeyPin GPIO controlling the BG77 PWRKEY input.
     */
    Bg77Mqtt(Print &pcSerial, Uart &modemSerial, uint32_t pwrKeyPin);

    /**
     * @brief Initializes serial and GPIO resources used by the modem driver.
     */
    void begin();
    /**
     * @brief Runs one service iteration for modem input and reconnection logic.
     */
    void loop();

    /**
     * @brief Performs a complete modem, network, and MQTT initialization.
     *
     * @return true if initialization reaches MQTT subscription state, false otherwise.
     */
    bool init();
    /**
     * @brief Ensures that the modem, LTE network, and MQTT session are usable.
     *
     * @return true if the connection is usable or restored, false otherwise.
     */
    bool ensureConnection();
    /**
     * @brief Ensures LTE registration, packet attachment, and IP availability.
     *
     * @return true if network access is available, false otherwise.
     */
    bool ensureNetwork();
    /**
     * @brief Ensures the MQTT session is connected.
     *
     * @return true if MQTT is connected or reconnected, false otherwise.
     */
    bool ensureMqtt();
    /**
     * @brief Reconnects the MQTT session using the current network context.
     *
     * @return true if MQTT reconnect succeeds, false otherwise.
     */
    bool mqttReconnect();

    /**
     * @brief Publishes a payload to the default publish topic.
     *
     * @param[in] payload Payload string to publish.
     * @return true if the publish command succeeds, false otherwise.
     */
    bool publish(const String &payload);
    /**
     * @brief Publishes a payload to a specific topic.
     *
     * @param[in] topic MQTT topic.
     * @param[in] payload Payload string to publish.
     * @return true if the publish command succeeds, false otherwise.
     */
    bool publishToTopic(const String &topic, const String &payload);
    /**
     * @brief Publishes an acknowledgment payload to the ACK topic.
     *
     * @param[in] payload Payload string to publish.
     * @return true if the publish command succeeds, false otherwise.
     */
    bool publishAck(const String &payload);

    /**
     * @brief Handles one modem-related console command line.
     *
     * @param[in] line Command line text.
     */
    void handleConsoleLine(const String &line);

    /**
     * @brief Returns the cached LTE registration state.
     *
     * @return true if the modem is considered network registered.
     */
    bool isNetworkRegistered() const;
    /**
     * @brief Returns the cached packet attachment state.
     *
     * @return true if packet service is considered attached.
     */
    bool isPacketAttached() const;
    /**
     * @brief Returns the cached MQTT connection state.
     *
     * @return true if MQTT is considered connected.
     */
    bool isMqttConnected() const;
    /**
     * @brief Sets the MQTT keepalive value used on the modem.
     *
     * @param[in] keepAliveSec Keepalive interval in seconds.
     */
    void setKeepAlive(uint16_t keepAliveSec);

    /**
     * @brief Requests the network date/time string from the modem.
     *
     * @return Raw or processed network date/time string depending on implementation state.
     */
    String getNetworkDateTime();
    /**
     * @brief Returns the network date/time formatted as ISO-8601 when available.
     *
     * @return ISO-8601 date/time string, or an implementation-defined empty/fallback string.
     */
    String getNetworkDateTimeIso8601();
    /**
     * @brief Converts a BG77 +QLTS date/time response into ISO-8601 format.
     *
     * @param[in] qlts Raw +QLTS response text.
     * @return Converted ISO-8601 string, or an implementation-defined fallback.
     */
    String convertQltsToIso8601(const String &qlts) const;

    /**
     * @brief Indicates whether an MQTT message is queued for consumption.
     *
     * @return true if a queued message exists, false otherwise.
     */
    bool hasNewMqttMessage() const;
    /**
     * @brief Returns the oldest queued MQTT message without removing it.
     *
     * @return Queued message text, or an empty string when the queue is empty.
     */
    String getLastMqttMessage() const;
    /**
     * @brief Removes the oldest queued MQTT message.
     */
    void clearLastMqttMessage();
    /**
     * @brief Consumes the one-shot just-connected flag.
     *
     * @return true once after a successful connection event, false otherwise.
     */
    bool consumeJustConnected();
    /**
     * @brief Sends a BG77 PWRKEY pulse.
     */
    void bg77PowerPulse();
    /**
     * @brief Publishes a payload to a C-string topic.
     *
     * @param[in] payload Payload string to publish.
     * @param[in] topic MQTT topic as a C string.
     * @return true if the publish command succeeds, false otherwise.
     */
    bool publish(const String &payload, const char *topic);
    /**
     * @brief Returns a compact LTE status string.
     *
     * @return Cached LTE status text.
     */
    String getLteStatus() const; //@@@ for mqtt connection

private:
    Print &_pc;          ///< Diagnostic output stream.
    Uart &_modem;        ///< UART connected to the BG77 modem.
    uint32_t _pwrKeyPin; ///< GPIO used to drive the BG77 PWRKEY input.

    String _modemLine; ///< Current line being assembled from modem input.

    bool _networkRegistered; ///< Cached LTE registration state.
    bool _packetAttached;    ///< Cached packet attachment state.
    bool _mqttSocketOpen;    ///< Cached MQTT socket-open state.
    bool _mqttConnected;     ///< Cached MQTT connection state.
    bool _justConnected;     ///< One-shot flag set after a successful connection.
    /**
     * @brief Cached compact LTE status text.
     */
    String _lteStatus;          //@@@@ for mqtt connection
    uint16_t _mqttKeepAliveSec; ///< MQTT keepalive interval in seconds.

    uint32_t _lastMqttOkMs;         ///< millis() timestamp of the last successful MQTT activity.
    uint32_t _nextReconnectAtMs;    ///< millis() timestamp before which reconnect is deferred.
    uint32_t _disconnectedSinceMs;  ///< millis() timestamp when MQTT disconnection was first noticed.
    uint8_t _mqttReconnectFailures; ///< Consecutive MQTT reconnect failure count.
    bool _bootPowerCycleDone;       ///< true after the boot-time modem power cycle was attempted.

    static constexpr uint32_t MAX_DISCONNECTED_MS = 30UL * 60UL * 1000UL;       ///< Maximum tolerated disconnected duration.
    static constexpr uint32_t MQTT_ACTIVITY_WATCHDOG_MS = 20UL * 60UL * 1000UL; ///< Maximum tolerated MQTT inactivity duration.
    static constexpr uint8_t MQTT_QUEUE_SIZE = 4;                               ///< Number of inbound MQTT messages retained.
    String _mqttQueue[MQTT_QUEUE_SIZE];                                         ///< Ring buffer for inbound MQTT payloads.
    uint8_t _mqttQueueHead;                                                     ///< Ring buffer read index.
    uint8_t _mqttQueueTail;                                                     ///< Ring buffer write index.
    uint8_t _mqttQueueCount;                                                    ///< Number of queued MQTT payloads.

    /**
     * @brief Writes an informational diagnostic line.
     *
     * @param[in] msg Message text.
     */
    void logInfo(const String &msg);
    /**
     * @brief Writes a warning diagnostic line.
     *
     * @param[in] msg Message text.
     */
    void logWarn(const String &msg);
    /**
     * @brief Writes an error diagnostic line.
     *
     * @param[in] msg Message text.
     */
    void logError(const String &msg);

    /**
     * @brief Discards pending bytes from the modem UART.
     */
    void clearModemInput();
    /**
     * @brief Prints the cached modem state for diagnostics.
     *
     * @param[in] origin Label describing the event that triggered the snapshot.
     */
    void printStateSnapshot(const String &origin);
    /**
     * @brief Processes one complete line received from the modem.
     *
     * @param[in] line Modem line without line-ending characters.
     */
    void handleModemLine(const String &line);
    /**
     * @brief Services modem input for a bounded duration.
     *
     * @param[in] durationMs Duration to pump modem input, in milliseconds.
     */
    void pumpModem(uint32_t durationMs);

    /**
     * @brief Sends an AT command and waits for one of two expected tokens.
     *
     * @param[in] cmd AT command without trailing CR/LF.
     * @param[in] token1 First expected response token, or nullptr.
     * @param[in] token2 Second expected response token, or nullptr.
     * @param[in] timeoutMs Maximum wait time, in milliseconds.
     * @param[out] fullResponse Optional full response accumulator.
     * @return true if an expected token is observed, false on timeout or modem error.
     */
    bool sendATWaitFor(const String &cmd, const char *token1, const char *token2, uint32_t timeoutMs, String *fullResponse = nullptr);
    /**
     * @brief Sends an AT command and waits for OK.
     *
     * @param[in] cmd AT command without trailing CR/LF.
     * @param[in] timeoutMs Maximum wait time, in milliseconds.
     * @return true if OK is received, false otherwise.
     */
    bool sendATOK(const String &cmd, uint32_t timeoutMs = 5000);

    /**
     * @brief Checks whether the modem responds to AT.
     *
     * @return true if the modem responds, false otherwise.
     */
    bool isModemAlive();
    /**
     * @brief Queries and updates LTE registration state.
     *
     * @return true if the query completed, false otherwise.
     */
    bool updateRegistration();
    /**
     * @brief Waits until the modem reports network registration.
     *
     * @param[in] timeoutMs Maximum wait time, in milliseconds.
     * @return true if registration is reached, false otherwise.
     */
    bool waitForRegistration(uint32_t timeoutMs);
    /**
     * @brief Queries and updates packet attachment state.
     *
     * @return true if the query completed, false otherwise.
     */
    bool updatePacketAttach();
    /**
     * @brief Checks whether the modem has an IP address.
     *
     * @return true if an IP address is reported, false otherwise.
     */
    bool hasIPAddress();

    /**
     * @brief Waits for the modem to become AT-responsive.
     *
     * @param[in] timeoutMs Maximum wait time, in milliseconds.
     * @return true if the modem becomes responsive, false otherwise.
     */
    bool waitModemReady(uint32_t timeoutMs);
    /**
     * @brief Runs post-power-on modem preparation commands.
     *
     * @return true if preparation succeeds, false otherwise.
     */
    bool prepareModemAfterPowerOn();
    /**
     * @brief Performs a BG77 power cycle sequence.
     *
     * @return true if the modem becomes responsive after the sequence, false otherwise.
     */
    bool bg77PowerCycle();
    /**
     * @brief Attempts one LTE network profile.
     *
     * @param[in] name Diagnostic profile name.
     * @param[in] plmn PLMN string, or nullptr depending on implementation.
     * @param[in] act Access technology code.
     * @param[in] iotopmode BG77 IoT operation mode value.
     * @return true if the profile reaches usable network state, false otherwise.
     */
    bool tryNetworkProfile(const char *name, const char *plmn, int act, int iotopmode);

    /**
     * @brief Applies the MQTT keepalive value through modem configuration.
     *
     * @param[in] keepAliveSec Keepalive interval in seconds.
     * @return true if the command succeeds, false otherwise.
     */
    bool setMQTTKeepAliveInternal(uint16_t keepAliveSec);
    /**
     * @brief Disconnects MQTT and closes the modem MQTT socket.
     */
    void mqttDisconnectClose();
    /**
     * @brief Subscribes to the configured MQTT command topic.
     *
     * @return true if subscription succeeds, false otherwise.
     */
    bool mqttSubscribe();

    /**
     * @brief Marks MQTT state as lost and records diagnostic information.
     *
     * @param[in] reason Human-readable reason.
     */
    void markMqttLost(const String &reason);
    /**
     * @brief Computes the current reconnect backoff delay.
     *
     * @return Delay in milliseconds.
     */
    uint32_t reconnectBackoffMs() const;
};
