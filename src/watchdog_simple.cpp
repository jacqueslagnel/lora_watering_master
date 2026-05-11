/**
 * @file watchdog_simple.cpp
 * @brief nRF52 watchdog helper implementation.
 *
 * This file configures the NRF_WDT peripheral, starts it once, and exposes a
 * feed function used by the application main loop.
 */
#include "watchdog_simple.h"

#include <nrf.h>

/**
 * @brief Tracks whether watchdogBegin() has already started the watchdog.
 */
static bool g_wdtStarted = false;

/**
 * @brief Initializes and starts the nRF watchdog peripheral.
 *
 * @post g_wdtStarted is true after the watchdog is started.
 * @warning The hardware watchdog cannot be stopped after TASKS_START is set.
 */
void watchdogBegin(uint32_t timeoutMs)
{
    if (g_wdtStarted)
    {
        return;
    }

    // WDT clock = 32768 Hz
    uint64_t ticks = ((uint64_t)timeoutMs * 32768ULL) / 1000ULL;

    if (ticks < 15ULL)
    {
        ticks = 15ULL;
    }
    if (ticks > 0xFFFFFFFFULL)
    {
        ticks = 0xFFFFFFFFULL;
    }

    // Run in sleep, pause when halted by debugger
    NRF_WDT->CONFIG =
        (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
        (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

    NRF_WDT->CRV = (uint32_t)ticks;

    // Enable reload register RR0
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;

    // Initial feed before start
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    // Start watchdog
    NRF_WDT->TASKS_START = 1;

    g_wdtStarted = true;
}

/**
 * @brief Reloads watchdog channel RR0 when the watchdog is active.
 */
void watchdogFeed()
{
    if (!g_wdtStarted)
    {
        return;
    }

    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

/**
 * @brief Returns the logical watchdog started state tracked by this module.
 *
 * @return true if watchdogBegin() has started the watchdog, false otherwise.
 */
bool watchdogIsRunning()
{
    return g_wdtStarted;
}
