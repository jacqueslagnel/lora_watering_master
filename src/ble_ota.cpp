/**
 * @file ble_ota.cpp
 * @brief BLE OTA boot-mode implementation.
 *
 * This file implements the BleOta namespace by storing an OTA boot marker in
 * the nRF power retention register, configuring Bluefruit DFU advertising, and
 * resetting the MCU when the OTA advertising window expires.
 */
#include "ble_ota.h"

#include <Arduino.h>
#include <bluefruit.h>
#include <nrf.h>

namespace BleOta
{
    /**
     * @brief Magic value stored in GPREGRET to request OTA mode after reset.
     */
    const uint8_t OTA_BOOT_MAGIC = 0xA5;

    /**
     * @brief Bluefruit DFU service object.
     */
    static BLEDfu bledfu;
    /**
     * @brief true while BLE OTA mode is active.
     */
    static bool g_active = false;
    /**
     * @brief millis() timestamp when OTA mode started.
     */
    static uint32_t g_startedMs = 0;

    // Version 1 simple:
    // fixed OTA window after reboot
    /**
     * @brief Fixed BLE advertising window used by this implementation, in milliseconds.
     */
    static const uint32_t OTA_ADVERTISING_WINDOW_MS = DURATION_DEFAULT_SEC * 1000UL;

    /**
     * @brief Bluefruit connection callback.
     *
     * @param[in] conn_handle Connection handle provided by the BLE stack.
     */
    static void connect_callback(uint16_t conn_handle)
    {
        (void)conn_handle;
        Serial.println("BLE client connected");
    }

    /**
     * @brief Bluefruit disconnection callback.
     *
     * @param[in] conn_handle Connection handle provided by the BLE stack.
     * @param[in] reason BLE disconnection reason.
     */
    static void disconnect_callback(uint16_t conn_handle, uint8_t reason)
    {
        (void)conn_handle;
        (void)reason;
        Serial.println("BLE client disconnected");
    }

    /**
     * @brief Clamps a requested OTA duration to the supported range.
     *
     * @return durationSec when it is valid, otherwise DURATION_DEFAULT_SEC.
     */
    uint32_t sanitizeDurationSec(uint32_t durationSec)
    {
        if (durationSec < DURATION_MIN_SEC || durationSec > DURATION_MAX_SEC)
        {
            return DURATION_DEFAULT_SEC;
        }
        return durationSec;
    }

    /**
     * @brief Stores the OTA boot marker in GPREGRET.
     *
     * @note The current implementation does not persist durationSec across reset.
     */
    void requestBoot(uint32_t durationSec)
    {
        (void)durationSec; // version 1: duration not yet persisted across reset
        NRF_POWER->GPREGRET = OTA_BOOT_MAGIC;
    }

    /**
     * @brief Checks whether GPREGRET contains the OTA boot marker.
     *
     * @return true if OTA mode was requested for this boot, false otherwise.
     */
    bool isRequestedAtBoot()
    {
        return (NRF_POWER->GPREGRET == OTA_BOOT_MAGIC);
    }

    /**
     * @brief Clears the OTA boot marker from GPREGRET.
     */
    void clearBootRequest()
    {
        NRF_POWER->GPREGRET = 0;
    }

    /**
     * @brief Starts BLE DFU advertising under the provided device name.
     *
     * @post g_active is true and g_startedMs contains the start timestamp.
     */
    void begin(const char *deviceName)
    {
        Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
        Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

        Bluefruit.begin(1, 0);
        Bluefruit.setTxPower(4);
        Bluefruit.setName(deviceName);

        Bluefruit.Periph.setConnectCallback(connect_callback);
        Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

        bledfu.begin();

        Bluefruit.Advertising.stop();
        Bluefruit.Advertising.clearData();
        Bluefruit.ScanResponse.clearData();

        Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
        Bluefruit.Advertising.addTxPower();
        Bluefruit.Advertising.addName();

        Bluefruit.ScanResponse.addName();

        Bluefruit.Advertising.restartOnDisconnect(true);
        Bluefruit.Advertising.setInterval(32, 244);
        Bluefruit.Advertising.setFastTimeout(30);
        Bluefruit.Advertising.start(0);

        g_active = true;
        g_startedMs = millis();

        Serial.println("BLE OTA mode started");
    }

    /**
     * @brief Services OTA timeout handling.
     *
     * If OTA mode is active and the fixed advertising window has elapsed, the
     * MCU is reset through NVIC_SystemReset().
     */
    void loop()
    {
        if (!g_active)
        {
            return;
        }

        if ((uint32_t)(millis() - g_startedMs) >= OTA_ADVERTISING_WINDOW_MS)
        {
            Serial.println("BLE OTA timeout, rebooting");
            delay(100);
            NVIC_SystemReset();
        }
    }

    /**
     * @brief Returns whether OTA mode is currently active.
     *
     * @return true if BLE OTA mode is active, false otherwise.
     */
    bool isActive()
    {
        return g_active;
    }
}
