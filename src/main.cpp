/**
 * @file main.cpp
 * @brief Arduino entry point for the LoRa master with LTE/MQTT and BLE OTA.
 *
 * This file wires together the selected LTE modem implementation, the LoRa P2P
 * master, the watchdog, battery measurement, periodic heartbeat publishing, and
 * the optional BLE OTA boot mode. Runtime work is driven from setup() and loop().
 */
#include <Arduino.h>
#ifdef _VARIANT_RAK4630_
#include <Adafruit_TinyUSB.h>
#endif

/**
 * @brief Numeric selector for the SIM7080 modem implementation.
 */
#define MODEM_SIM7080 1
/**
 * @brief Numeric selector for the BG77 modem implementation.
 */
#define MODEM_BG77 2

/**
 * @brief Active modem selection used by the build.
 */
#define MODEM MODEM_SIM7080

#if MODEM == MODEM_SIM7080
#include "sim7080_mqtt.h"
#else
#include "bg77_mqtt.h"
#endif

#include "lora_master.h"
#include "watchdog_simple.h"

#include <bluefruit.h>
#include "ble_ota.h"

/**
 * @brief USB/serial console stream.
 */
#define PC_SERIAL Serial
/**
 * @brief UART stream connected to the LTE modem.
 */
#define MODEM_SERIAL Serial1
/**
 * @brief GPIO used as the modem PWRKEY control pin.
 */
#define SIM7080_PWRKEY_PIN WB_IO1

/**
 * @brief Analog pin used for battery voltage measurement.
 */
#define PIN_VBAT A0
/**
 * @brief Runtime battery ADC pin used by readVBAT().
 */
static uint32_t vbat_pin = PIN_VBAT;

/**
 * @brief ADC millivolts per LSB before divider compensation.
 */
#define VBAT_MV_PER_LSB (0.73242188F)
/**
 * @brief Compensation factor applied to the battery divider measurement.
 */
#define VBAT_DIVIDER_COMP (1.7250F)
/**
 * @brief Effective millivolts per ADC LSB after divider compensation.
 */
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

/**
 * @brief Periodic heartbeat interval, in milliseconds.
 */
static const uint32_t MQTT_PERIOD_MS = 10UL * 60UL * 1000UL;
#if MODEM == MODEM_SIM7080
/**
 * @brief Selected LTE modem instance used by the application.
 */
Sim7080Mqtt lte_modem(PC_SERIAL, MODEM_SERIAL, SIM7080_PWRKEY_PIN);
#else
/**
 * @brief Selected LTE modem instance used by the application.
 */
Bg77Mqtt lte_modem(PC_SERIAL, MODEM_SERIAL, SIM7080_PWRKEY_PIN);
#endif

/**
 * @brief LoRa P2P master instance.
 */
LoRaMaster lora;

/**
 * @brief millis() timestamp for the next heartbeat publication.
 */
static uint32_t nextMqttPeriodicAt = 0;
/**
 * @brief Monotonic heartbeat counter included in MQTT heartbeat payloads.
 */
static uint32_t heartbeatCounter = 0;

/**
 * @brief Reads the compensated battery voltage.
 *
 * @return Battery voltage in millivolts as a floating-point value.
 */
static float readVBAT(void);
/**
 * @brief Reads the battery voltage and returns a bounded integer value.
 *
 * @return Battery voltage in millivolts, clamped to uint16_t range.
 */
static uint16_t readBatteryMilliVolts();
/**
 * @brief Publishes a payload through the active LTE modem.
 *
 * @param[in] payload Payload to publish.
 * @return true if the modem publish succeeds, false otherwise.
 */
static bool publishToMqtt(const String &payload);
/**
 * @brief Publishes the periodic master heartbeat.
 */
static void publishHeartbeat();
/**
 * @brief Consumes one inbound MQTT command and forwards it to LoRaMaster.
 */
static void processIncomingMqtt();
/**
 * @brief Reboots the master MCU.
 */
static void rebootMaster();

/**
 * @brief Arduino setup entry point.
 *
 * Initializes watchdog, console, optional BLE OTA mode, ADC battery measurement,
 * LTE modem service, and LoRa master service.
 *
 * @post In normal boot, lte_modem.begin() and lora.begin() have been called.
 * @post In OTA boot, BLE OTA mode is started and normal modem/radio init is skipped.
 */
void setup()
{
    NRF_RADIO->POWER = 0;

    watchdogBegin(20000);
    watchdogFeed();

    PC_SERIAL.begin(115200);
    uint32_t waitStart = millis();
    while (!PC_SERIAL && (millis() - waitStart < 5000))
    {
        watchdogFeed();
        delay(10);
    }

    // --------------- OTA boot -------------------------
    if (BleOta::isRequestedAtBoot())
    {
        BleOta::clearBootRequest();
        PC_SERIAL.println("Booting in BLE OTA mode");
        BleOta::begin("MASTER_OTA");
        return;
    }
    // --------------- END OTA boot -------------------------

    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    delay(5);
    readVBAT();
    delay(5);
    readVBAT();

    /*     PC_SERIAL.println("-> BEGIN modemPowerPulse");
        lte_modem.modemPowerPulse();
        delay(1000);
        PC_SERIAL.println("-> END modemPowerPulse");
        PC_SERIAL.flush(); */

    lte_modem.begin();

    lora.setPublishCallback(publishToMqtt);
    lora.begin();
    lora.setMasterBatteryReader(readBatteryMilliVolts);
    lora.setMasterRebootHandler(rebootMaster);
    // LTE/MQTT init is handled from lte_modem.loop().
    // tryInitialRtcSync() is intentionally not called here because modem/network may not be ready yet

    // wait 60 sec before first heartbeat
    nextMqttPeriodicAt = millis() + 60000UL;
}

/**
 * @brief Arduino main loop entry point.
 *
 * Services BLE OTA mode when active, feeds the watchdog, routes serial console
 * commands, runs LTE and LoRa state machines, processes inbound MQTT commands,
 * and publishes periodic heartbeat payloads.
 */
void loop()
{
    // ------------ OTA mode ---------------------
    if (BleOta::isActive())
    {
        BleOta::loop();
        watchdogFeed();
        __WFI();
        return;
    }
    // ------------ END OTA mode -----------------

    watchdogFeed();

    if (PC_SERIAL.available())
    {
        String line = PC_SERIAL.readStringUntil('\n');
        line.trim();

        String lower = line;
        lower.toLowerCase();

        if (lower == "pwr" ||
            lower == "reinit" ||
            lower == "disc" ||
            lower.startsWith("ka ") ||
            lower.startsWith("at"))
        {
            lte_modem.handleConsoleLine(line);
        }
        else
        {
            lora.handleConsoleLine(line);
        }
    }

    lte_modem.loop();
    // @@@@ envoie mqtt on connect
    if (lte_modem.consumeJustConnected())
    {
        lte_modem.publish("LTE_CONNECTED;" + lte_modem.getLteStatus());
        lora.republishAllNodePresence();
    }
    processIncomingMqtt();

    lora.loop();

    uint32_t now = millis();
    if ((int32_t)(now - nextMqttPeriodicAt) >= 0)
    {
        publishHeartbeat();
        nextMqttPeriodicAt = millis() + MQTT_PERIOD_MS;
    }

    watchdogFeed();
    __WFI();
}

// ##############################################################################################
// ################################ functions ####################################################
// ##############################################################################################

/**
 * @brief Reads and averages battery ADC samples.
 *
 * Eight ADC samples are taken from vbat_pin and averaged before applying the
 * configured conversion factor.
 *
 * @return Compensated battery voltage in millivolts.
 * @pre The ADC reference and resolution must be configured by setup().
 */
static float readVBAT(void)
{
    uint32_t acc = 0;
    const uint8_t n = 8;
    for (uint8_t i = 0; i < n; i++)
    {
        acc += (uint32_t)analogRead(vbat_pin);
        delay(2);
    }
    float raw = (float)acc / n;
    return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Reads the battery voltage as an unsigned 16-bit millivolt value.
 *
 * @return Battery voltage rounded to millivolts and clamped to [0, 65535].
 */
static uint16_t readBatteryMilliVolts()
{
    float vbat_mv = readVBAT();
    if (vbat_mv < 0.0f)
        vbat_mv = 0.0f;
    if (vbat_mv > 65535.0f)
        vbat_mv = 65535.0f;
    return (uint16_t)(vbat_mv + 0.5f);
}

/**
 * @brief Publishes a heartbeat payload over MQTT.
 *
 * The payload includes the heartbeat counter, master battery voltage, online
 * node count, and CSV list of online node identifiers.
 *
 * @post heartbeatCounter is incremented.
 */
static void publishHeartbeat()
{
    heartbeatCounter++;

    String payload = "H;";
    payload += String(heartbeatCounter);
    payload += ";";
    payload += String(readBatteryMilliVolts());
    payload += ";";
    payload += String(lora.getOnlineNodeCount());
    payload += ";";
    payload += lora.getOnlineNodeListCsv();

    lte_modem.publish(payload);
}

/**
 * @brief Forwards the oldest queued MQTT command to the LoRa command parser.
 *
 * @post If a queued MQTT message existed, it is removed from the LTE modem queue.
 */
static void processIncomingMqtt()
{
    if (!lte_modem.hasNewMqttMessage())
    {
        return;
    }

    String msg = lte_modem.getLastMqttMessage();
    lora.handleConsoleLine("mqtt " + msg);
    lte_modem.clearLastMqttMessage();
}

/**
 * @brief Publishes a payload through the active LTE modem instance.
 *
 * @return true if lte_modem accepts the publication, false otherwise.
 */
static bool publishToMqtt(const String &payload)
{
    return lte_modem.publish(payload);
}

/**
 * @brief Resets the master MCU through the NVIC system reset request.
 *
 * @warning This function does not return on normal hardware.
 */
static void rebootMaster()
{
    NVIC_SystemReset();
}
