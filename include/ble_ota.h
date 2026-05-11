/**
 * @file ble_ota.h
 * @brief Interface for the BLE OTA mode used at master startup.
 *
 * This file declares the functions used to request a boot into BLE OTA update
 * mode, detect that request at startup, and run the OTA service loop. Accepted
 * OTA durations are bounded by constants exposed in the BleOta namespace.
 */
#pragma once

#include <Arduino.h>

/**
 * @namespace BleOta
 * @brief Functions and constants for firmware update over BLE.
 *
 * The namespace groups the OTA boot request state, OTA window duration handling,
 * and the entry points called by the main program.
 */
namespace BleOta
{
    /**
     * @brief OTA boot request marker stored by the implementation.
     */
    extern const uint8_t OTA_BOOT_MAGIC;

    /**
     * @brief Minimum allowed OTA window duration, in seconds.
     */
    static constexpr uint32_t DURATION_MIN_SEC = 60UL;
    /**
     * @brief Maximum allowed OTA window duration, in seconds.
     */
    static constexpr uint32_t DURATION_MAX_SEC = 1800UL;
    /**
     * @brief OTA duration used when no explicit valid duration is provided.
     */
    static constexpr uint32_t DURATION_DEFAULT_SEC = 300UL;

    /**
     * @brief Clamps an OTA duration to the accepted interval.
     *
     * @param[in] durationSec Requested duration, in seconds.
     * @return Corrected duration in the [DURATION_MIN_SEC, DURATION_MAX_SEC] range.
     */
    uint32_t sanitizeDurationSec(uint32_t durationSec);

    /**
     * @brief Requests a later restart into BLE OTA mode.
     *
     * @param[in] durationSec Requested OTA window duration, in seconds.
     * @post An OTA boot request is made visible to isRequestedAtBoot().
     */
    void requestBoot(uint32_t durationSec);
    /**
     * @brief Indicates whether a BLE OTA boot was requested.
     *
     * @return true if an OTA request is present at startup, false otherwise.
     */
    bool isRequestedAtBoot();
    /**
     * @brief Clears the OTA boot request persisted by requestBoot().
     *
     * @post isRequestedAtBoot() returns false until a new request is stored.
     */
    void clearBootRequest();

    /**
     * @brief Initializes the BLE OTA service.
     *
     * @param[in] deviceName BLE name advertised by the device.
     * @pre deviceName must point to a valid C string.
     * @post OTA mode may become active depending on the implementation.
     */
    void begin(const char *deviceName);
    /**
     * @brief Runs one service iteration of the BLE OTA mode.
     *
     * @pre begin() must have been called before this loop is used.
     */
    void loop();
    /**
     * @brief Indicates whether BLE OTA mode is active.
     *
     * @return true if the main loop must delegate to the OTA service, false otherwise.
     */
    bool isActive();
}
