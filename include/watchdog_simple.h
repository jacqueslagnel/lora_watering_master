/**
 * @file watchdog_simple.h
 * @brief Minimal interface for driving the nRF52 hardware watchdog.
 *
 * This file exposes initialization, feeding, and logical state reporting for the
 * watchdog used by the firmware main loop.
 */
#pragma once

#include <Arduino.h>

/**
 * @brief Initializes and starts the hardware watchdog.
 *
 * @param[in] timeoutMs Maximum time without a feed before reset, in milliseconds.
 * @post watchdogIsRunning() returns true if initialization was performed.
 * @warning Once the nRF watchdog is started, it generally cannot be stopped
 * before a hardware reset.
 */
void watchdogBegin(uint32_t timeoutMs = 20000); // 20000 =20 sec
/**
 * @brief Feeds the watchdog if it has been started.
 *
 * @note The call is ignored if watchdogBegin() has not been executed yet.
 */
void watchdogFeed();
/**
 * @brief Indicates whether the watchdog was started through this interface.
 *
 * @return true if watchdogBegin() has already started the watchdog, false otherwise.
 */
bool watchdogIsRunning();
