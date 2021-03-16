/**
 * CySump
 * v1.6
 * 
 * Author:
 *  Cyrus Brunner <cyrusbuilt at gmail dot com>
 * 
 * This is the firmware for the CySump IoT Sump Pump Controller. This firmware
 * specifically targets the Adafruit Huzzah ESP8266, but could easily be
 * adapted to work on any ESP8266 MCU.  It should be relatively easy to adapt
 * to work on ESP32 modules as well, although there would need to be some
 * dependency changes.
 */

// TODO Implement FS checks in self-diags

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "config.h"
#include <WiFiClient.h>
#include <FS.h>
#include <time.h>
#include "Buzzer.h"
#include "LED.h"
#include "Relay.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "ArduinoJson.h"
#include "ESP8266Ping.h"
#include "PubSubClient.h"
#include "TelemetryHelper.h"
#include "WaterLevelSensor.h"
#include "Console.h"

#define FIRMWARE_VERSION "1.6"

// Pin definitions
#define PIN_WIFI_LED 2
#define PIN_ALARM_LED 4
#define PIN_PUMP_LED 5
#define PIN_RELAY 15
#define PIN_BUZZER 16
#define PIN_LEVEL_SENSOR A0

// Forward declarations
void onRelayStateChange(RelayInfo* sender);
void onCheckWiFi();
void failSafe();
void onCheckMqtt();
void onCheckSensors();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void onSyncClock();

// Global vars
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WaterLevelSensor sensor(PIN_LEVEL_SENSOR);
Relay pumpRelay(PIN_RELAY, onRelayStateChange, "SUMP_PUMP");
LED alarmLED(PIN_ALARM_LED, NULL);
LED wifiLED(PIN_WIFI_LED, NULL);
LED pumpLED(PIN_PUMP_LED, NULL);
Buzzer alarmBuzzer(PIN_BUZZER, NULL, "AlarmBuzzer");
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tSyncClock(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Task tCheckSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onCheckSensors);
Scheduler taskMan;
config_t config;
float lastDepth = 0;
volatile int percentFull = 0;
bool filesystemMounted = false;
bool manualPumpMode = false;
volatile SystemState sysState = SystemState::BOOTING;
bool alarmDisabled = false;

// TODO Maybe add some logic to measure how fast the water level is rising.
// If rising too fast, especially while the pump is running, this could be
// another alarm condition. Or even more importantly, if the pump is NOT
// running when we expect it to be, that would also be a cause for concern.
// However, I'm not sure how to test that other than to see if the water
// level goes down during pump operation. If it doesn't, then either the
// pump is NOT actually running, or again... the pit is filling faster
// than we can pump. This is a highly unlikely scenario though. You would
// have to have a pretty MAJOR flood problem if the water pump is not able
// to keep up with the incoming water. More than likely, this would simply
// indicate pump failure.


/**
 * Get and print the pump status.
 * @return The pump status ("OFF" or "ON")
 */
String pumpStatus() {
    String state = "OFF";
    if (pumpRelay.isClosed()) {
        state = "ON";
    }

    Serial.print(F("INFO: Pump status = "));
    Serial.println(state);
    return state;
}

/**
 * Gets the pit state based on the specified water level percentage.
 * @param percentFull The percentage of the pit that is full of water.
 * @return 
 */
String pitState(int percentFull) {
    String state = "UNKNOWN";
    if (percentFull == 0) {
        state = "EMPTY";
    }
    else if (percentFull <= 50) {
        state = "LOW";
    }
    else if (percentFull > 50 && percentFull < 75) {
        state = "WARN";
    }
    else if (percentFull >= 75 && percentFull < 99) {
        state = "CRITICAL";
    }
    else if (percentFull >= 99) {
        state = "FLOOD";
    }

    Serial.print(F("INFO: *** Pit state: "));
    Serial.println(state);
    return state;
}

/**
 * Publishes the system state to the MQTT status channel and
 * blinks the WiFi status LED.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        wifiLED.on();

        uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;

        DynamicJsonDocument doc(freeMem);
        doc["client_id"] = config.hostname;
        doc["pumpState"] = pumpStatus();
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["waterLevelPercent"] = percentFull;
        doc["pitState"] = pitState(percentFull);
        doc["waterDepth"] = lastDepth;
        doc["alarmEnabled"] = alarmDisabled ? "OFF" : "ON";
        doc.shrinkToFit();

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    configTime(CLOCK_TIMEZONE * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(asctime(timeinfo));
}

/**
 * Handles the relay state change event. Simply changes the state of the
 * relay indicator LED.
 * @param sender The relay info.
 */
void onRelayStateChange(RelayInfo* sender) {
    if (sender->state == RelayState::RelayClosed) {
        pumpLED.on();
    }
    else {
        pumpLED.off();
    }
}

/**
 * Toggles the water pump.
 * @param run Set true to turn the pump on; false to turn it off.
 */
void togglePump(bool run) {
    if (run) {
        Serial.print(F("INFO: Activating pump @ "));
        pumpRelay.close();
    }
    else {
        Serial.print(F("INFO: Deactivating pump @ "));
        pumpRelay.open();
    }

    Serial.println(millis());
}

/**
 * Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    wifiLED.off();
    // TODO Should we do anything with the other LEDs?
    sysState = SystemState::NORMAL;
    publishSystemState();
}

/**
 * Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    WiFi.printDiag(Serial);
}

/**
 * Scan for available networks and dump each discovered network to the console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * Reboots the MCU after a 1 second delay.
 */
void reboot() {
    Serial.println(F("INFO: Rebooting... "));
    Serial.flush();
    delay(1000);
    ResetManager.softReset();
}

/**
 * Stores the in-memory configuration to a JSON file stored in SPIFFS.
 * If the file does not yet exist, it will be created (see CONFIG_FILE_PATH).
 * Errors will be reported to the serial console if the filesystem is not
 * mounted or if the file could not be opened for writing.
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    StaticJsonDocument<350> doc;
    doc["hostname"] = config.hostname;
    doc["useDHCP"] = config.useDhcp;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gw.toString();
    doc["subnetMask"] = config.sm.toString();
    doc["dnsServer"] = config.dns.toString();
    doc["wifiSSID"] = config.ssid;
    doc["wifiPassword"] = config.password;
    doc["mqttBroker"] = config.mqttBroker;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttControlChannel"] = config.mqttTopicControl;
    doc["mqttStatusChannel"] = config.mqttTopicStatus;
    doc["mqttUsername"] = config.mqttUsername;
    doc["mqttPassword"] = config.mqttPassword;
    doc["activatePercentFull"] = config.pumpActivatePercent;
    doc["deactivatePercent"] = config.pumpDeactivatePercent;
    doc["pitDepth"] = config.pitDepth;
    doc["alarmDepthInches"] = config.alarmDepthInches;
    #ifdef ENABLE_OTA
        doc["otaPort"] = config.otaPort;
        doc["otaPassword"] = config.otaPassword;
    #endif

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

void setConfigurationDefaults() {
    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = defHostname;
    config.ip = defaultIp;
    config.mqttBroker = MQTT_BROKER;
    config.mqttPassword = "";
    config.mqttPort = MQTT_PORT;
    config.mqttTopicControl = MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = MQTT_TOPIC_STATUS;
    config.mqttUsername = "";
    config.password = DEFAULT_PASSWORD;
    config.sm = defaultSm;
    config.ssid = DEFAULT_SSID;
    config.useDhcp = false;
    config.clockTimezone = CLOCK_TIMEZONE;
    config.dns = defaultDns;
    config.gw = defaultGw;
    config.pitDepth = PIT_DEPTH_INCHES;
    config.pumpActivatePercent = DEFAULT_PUMP_ACTIVATE_PERCENT;
    config.pumpDeactivatePercent = DEFAULT_PUMP_DEACTIVATE_PERCENT;
    config.alarmDepthInches = ALARM_DEPTH_INCHES;
    #ifdef ENABLE_OTA
        config.otaPassword = OTA_PASSWORD;
        config.otaPort = OTA_HOST_PORT;
    #endif
}

void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

/**
 * Loads the configuration from CONFIG_FILE_PATH into memory and uses that as
 * the running configuration. Will report errors to the serial console and
 * revert to the default configuration under the following conditions:
 * 1) The filesystem is not mounted.
 * 2) The config file does not exist in SPIFFS. In this case a new file
 * will be created and populated with the default configuration.
 * 3) The config file exists, but could not be opened for reading.
 * 4) The config file is too big ( > 1MB).
 * 5) The config file could not be deserialized to a JSON structure.
 */
void loadConfiguration() {
    memset(&config, 0, sizeof(config));

    Serial.print(F("INFO: Loading config file: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        configFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        configFile.close();
        return;
    }

    doc.shrinkToFit();
    configFile.close();

    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = doc.containsKey("hostname") ? doc["hostname"].as<String>() : defHostname;
    config.useDhcp = doc.containsKey("isDhcp") ? doc["isDhcp"].as<bool>() : false;

    if (doc.containsKey("ip")) {
        if (!config.ip.fromString(doc["ip"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.ip = defaultIp;
    }

    if (doc.containsKey("gateway")) {
        if (!config.gw.fromString(doc["gateway"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
        }
    }
    else {
        config.gw = defaultGw;
    }

    if (doc.containsKey("subnetmask")) {
        if (!config.sm.fromString(doc["subnetmask"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
        }
    }
    else {
        config.sm = defaultSm;
    }

    if (doc.containsKey("dns")) {
        if (!config.dns.fromString(doc["dns"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid DNS IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.dns = defaultDns;
    }

    config.ssid = doc.containsKey("wifiSSID") ? doc["wifiSSID"].as<String>() : DEFAULT_SSID;
    config.password = doc.containsKey("wifiPassword") ? doc["wifiPassword"].as<String>() : DEFAULT_PASSWORD;
    config.mqttBroker = doc.containsKey("mqttBroker") ? doc["mqttBroker"].as<String>() : MQTT_BROKER;
    config.mqttPort = doc.containsKey("mqttPort") ? doc["mqttPort"].as<int>() : MQTT_PORT;
    config.mqttTopicControl = doc.containsKey("mqttControlChannel") ? doc["mqttControlChannel"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusChannel") ? doc["mqttStatusChannel"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";

    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    config.alarmDepthInches = doc.containsKey("alarmDepthInches") ? doc["alarmDepthInches"].as<uint8_t>() : ALARM_DEPTH_INCHES;
    config.pitDepth = doc.containsKey("pitDepth") ? doc["pitDepth"].as<uint8_t>() : PIT_DEPTH_INCHES;
    config.pumpActivatePercent = doc.containsKey("activatePercentFull") ? doc["activatePercentFull"].as<uint8_t>() : DEFAULT_PUMP_ACTIVATE_PERCENT;
    config.pumpDeactivatePercent = doc.containsKey("deactivatePercentFull") ? doc["deactivatePercentFull"].as<uint8_t>() : DEFAULT_PUMP_DEACTIVATE_PERCENT; 

    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * Confirms with the user that they wish to do a factory restore. If so, then
 * clears the current configuration file in SPIFFS, then reboots. Upon reboot,
 * a new config file will be generated with default values.
 * @param fromSubmit Set true if request came from the configuration page.
 * Default is false.
 */
void doFactoryRestore(bool fromSubmit = false) {
    String str = "N";
    if (!fromSubmit) {
        Serial.println();
        Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)?"));
        Console.waitForUserInput();
        str = Console.getInputString();
    }
    
    if (str == "Y" || str == "y" || fromSubmit) {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete configuration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Filesystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * Attempts to re-connect to the MQTT broker if then connection is
 * currently broken.
 * @return true if a connection to the MQTT broker is either already
 * established, or was successfully re-established; Otherwise, false.
 * If the connection is re-established, then will also re-subscribe to
 * the status channel.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(config.mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(config.mqttPort);
        Serial.println(F("..."));

        bool didConnect = false;
        if (config.mqttUsername.length() > 0 && config.mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(config.hostname.c_str(), config.mqttUsername.c_str(), config.mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(config.hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(config.mqttTopicStatus);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }
    }

    return true;
}

/**
 * Callback method for checking the MQTT connection state and
 * reconnecting if necessary. If the connection is broken, and
 * reconnection fails, another attempt will be made after CHECK_MQTT_INTERVAL.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connection status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds."));
    }
}

/**
 * Processes control requests. Executes the specified command
 * if valid and intended for this client.
 * @param id The client ID the command is intended for.
 * @param cmd The command to execute.
 */
void handleControlRequest(String id, ControlCommand cmd) {
    id.toUpperCase();
    if (!id.equals(config.hostname)) {
        Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
        return;
    }

    if (sysState == SystemState::DISABLED &&
        cmd != ControlCommand::ENABLE) {
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ingoring command "));
        Serial.print((uint8_t)cmd);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (cmd) {
        case ControlCommand::ACTIVATE:
            manualPumpMode = true;
            togglePump(true);
            break;
        case ControlCommand::DEACTIVATE:
            manualPumpMode = false;
            togglePump(false);
            break;
        case ControlCommand::DISABLE:
            Serial.println(F("WARN: Disabling system."));
            sysState = SystemState::DISABLED;
            break;
        case ControlCommand::ENABLE:
            Serial.println(F("INFO: Enabling system."));
            sysState = SystemState::NORMAL;
            break;
        case ControlCommand::REBOOT:
            reboot();
            break;
        case ControlCommand::DISABLE_ALARM:
            Serial.println(F("WARN: Silencing alarm."));
            alarmDisabled = true;
            alarmBuzzer.off();
            break;
        case ControlCommand::ENABLE_ALARM:
            Serial.println(F("INFO: Alarm enabled."));
            alarmDisabled = false;
            break;
        case ControlCommand::REQUEST_STATUS:
            break;
        default:
            Serial.print(F("WARN: Unknown command: "));
            Serial.println((uint8_t)cmd);
            break;
    }

    publishSystemState();
}

/**
 * Callback method for handling incoming MQTT messages on the subscribed
 * channel(s). This will decode the message and process incoming commands.
 * @param topic The topic the message came in on.
 * @param payload The message payload. The payload is assumed to be in
 * JSON format and will attempt to parse the message into a JSON object.
 * @param length The payload (message) length.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);

    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    DynamicJsonDocument doc(freeMem);

    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    doc.shrinkToFit();
    String id = doc["client_id"].as<String>();
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();
    doc.clear();
    handleControlRequest(id, cmd);
}

/**
 * Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);
            if (!mdns.begin(config.hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(config.hostname, "ota", config.otaPort);
            #endif
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    setConfigurationDefaults();
    loadConfiguration();
}

/**
 * Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(onMqttMessage);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * Attempt to connect to the configured WiFi network. This will break any existing connection first.
 */
void connectWifi() {
    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (config.useDhcp) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(config.ip, config.gw, config.sm, config.dns);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(config.ssid, config.password);
    Serial.println(F("DEBUG: Waiting for connection..."));
    
    const int maxTries = 20;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        ESPCrashMonitor.iAmAlive();
        currentTry++;
        wifiLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        failSafe();
    }
    else {
        printNetworkInfo();
    }
}

/**
 * Runs diagnostics to test the network and attached components.
 */
void runDiagnostics() {
    Serial.println(F("INFO: Beginning self-diagnostics..."));
    Serial.print(F("DIAG: Testing network... "));
    if (wifiClient.connected()) {
        if (Ping.ping(config.gw)) {
            Serial.println(F("PASS"));
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Unable to ping gateway."));
        }
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: WiFi not connected to AP."));
    }

    Serial.print(F("DIAG: Testings LEDs... "));
    bool currentState = wifiLED.isOn();
    if (currentState) {
        wifiLED.off();
    }

    delay(500);
    bool ledResult = false;
    wifiLED.on();
    delay(500);
    ledResult = wifiLED.isOn();
    wifiLED.off();
    ledResult &= wifiLED.isOff();

    if (currentState) {
        delay(500);
        wifiLED.on();
    }

    currentState = alarmLED.isOn();
    if (currentState) {
        alarmLED.off();
    }

    delay(500);
    alarmLED.on();
    delay(500);
    ledResult &= alarmLED.isOn();
    alarmLED.off();
    ledResult &= alarmLED.isOff();
    if (currentState) {
        delay(500);
        alarmLED.on();
    }

    currentState = pumpLED.isOn();
    if (currentState) {
        pumpLED.off();
    }

    delay(500);
    pumpLED.on();
    delay(500);
    ledResult &= pumpLED.isOn();
    pumpLED.off();
    ledResult &= pumpLED.isOff();
    if (currentState) {
        delay(500);
        pumpLED.on();
    }

    if (ledResult) {
        Serial.println(F("PASS"));
    }
    else {
        Serial.println(F("FAIL"));
    }
    
    Serial.print(F("DIAG: Testing buzzer... "));
    bool buzzerResult = false;
    currentState = alarmBuzzer.isOn();
    if (currentState) {
        alarmBuzzer.off();
    }

    delay(500);
    alarmBuzzer.on();
    delay(500);
    buzzerResult = alarmBuzzer.isOn();
    alarmBuzzer.off();
    buzzerResult &= alarmBuzzer.isOff();
    if (currentState) {
        delay(500);
        alarmBuzzer.on();
    }

    if (buzzerResult) {
        Serial.println(F("PASS"));
    }
    else {
        Serial.println(F("FAIL"));
    }

    Serial.print(F("DIAG: Testing ranging sensor... "));
    float depth = sensor.read();
    if (depth > 0) {
        Serial.println(F("PASS"));
    }
    else {
        Serial.println(F("FAIL"));
    }
    
    Serial.print(F("DIAG: Testing pump... "));
    bool pumpResult = false;
    currentState = pumpRelay.isClosed();
    if (currentState) {
        pumpRelay.open();
    }

    delay(1000);
    pumpRelay.close();
    delay(1000);
    pumpResult = pumpRelay.isClosed();
    pumpRelay.open();
    pumpResult &= pumpRelay.isOpen();
    if (currentState) {
        delay(1000);
        pumpRelay.close();
    }

    if (pumpResult) {
        Serial.println(F("PASS"));
    }
    else {
        Serial.println(F("FAIL"));
    }

    Serial.println(F("DIAG: Getting WiFi diags... "));
    WiFi.printDiag(Serial);

    Serial.println(F("INFO: Self-diagnostics complete."));
}

/**
 * Simply report the pit depth in inches.
 */
void reportDepth() {
    // Initialze results to 0.
    float results[5];
    for (uint8_t thisReading = 0; thisReading < sizeof(results); thisReading++) {
        results[thisReading] = 0;
    }

    // It takes ~100ms to measure distance, but we need to do some
    // smoothing here, so we'll get 5 samples and average them, which
    // should take ~650ms to complete (including delay).
    float total = 0;
    for (uint8_t thisReading = 0; thisReading < sizeof(results); thisReading++) {
        results[thisReading] = sensor.getLevelInches();
        total += results[thisReading];
        delay(30);
    }

    float average = total / 5;
    Serial.print(F("INFO: Pit depth = "));
    Serial.print(average);
    Serial.println(F(" inches."));
}

/**
 * Enter fail-safe mode. This will suspend all tasks, disable relay activation,
 * and propmpt the user for configuration.
 */
void failSafe() {
    sysState = SystemState::DISABLED;
    publishSystemState();
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    alarmLED.on();
    wifiLED.on();
    pumpRelay.open();
    Console.enterCommandInterpreter();
}

/**
 * Initialize the fluid level sensor.
 */
void initDepthSensor() {
    Serial.print(F("INIT: Initializing depth sensor... "));
    sensor.begin();
    sensor.setTapeLengthCm(config.pitDepth);
    Serial.println(F("DONE"));
}

/**
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(config.ssid);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(config.otaPort);
            ArduinoOTA.setHostname(config.hostname.c_str());
            ArduinoOTA.setPassword(config.otaPassword.c_str());
            ArduinoOTA.onStart([]() {
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }

                sysState = SystemState::UPDATING;
                publishSystemState();
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater stopped."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
                wifiLED.blink(100);
                ESPCrashMonitor.iAmAlive();
                Serial.printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
            });
            ArduinoOTA.onError([](ota_error_t error) {
                // Handles OTA update errors.
                Serial.printf("ERROR: OTA update error [%u]: ", error);
                switch(error) {
                    case OTA_AUTH_ERROR:
                        Serial.println(F("Auth failed."));
                        break;
                    case OTA_BEGIN_ERROR:
                        Serial.println(F("Begin failed."));
                        break;
                    case OTA_CONNECT_ERROR:
                        Serial.println(F("Connect failed."));
                        break;
                    case OTA_RECEIVE_ERROR:
                        Serial.println(F("Receive failed."));
                        break;
                    case OTA_END_ERROR:
                        Serial.println(F("End failed."));
                        break;
                }
            });
            ArduinoOTA.begin();
            Serial.println(F("DONE"));
        }
        else {
            Serial.println(F("FAIL"));
        }
    #endif
}

/**
 * Initializes the RS232 serial interface.
 */
void initSerial() {
    Serial.setDebugOutput(true);
    Serial.begin(SERIAL_BAUD, SERIAL_8N1);
    Serial.println();
    Serial.println();
    Serial.print(F("INIT: CyGarage v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
    Serial.println();
}

/**
 * Initializes output components.
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing components... "));
    pumpLED.init();
    pumpLED.on();

    wifiLED.init();
    wifiLED.on();

    alarmLED.init();
    alarmLED.on();

    pumpRelay.init();
    pumpRelay.open();

    alarmBuzzer.init();
    
    delay(1000);
    Serial.println(F("DONE"));
}

/**
 * Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckSensors);
    taskMan.addTask(tCheckWifi);
    taskMan.addTask(tCheckMqtt);
    taskMan.addTask(tSyncClock);
    
    tCheckWifi.enableDelayed(30000);
    tCheckSensors.enable();
    tCheckMqtt.enableDelayed(1000);
    tSyncClock.enable();
    Serial.println(F("DONE"));
}

/**
 * Callback routine for checking WiFi connectivity.
 */
void onCheckWiFi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initOTA();
        }
    }
}

/**
 * Callback routine that checks the sensor inputs to determine door state.
 */
void onCheckSensors() {
    Serial.println(F("INFO: Checking sensor..."));
    float depthReading = sensor.getLevelInches();
    if (depthReading < 0) {
        depthReading = 0;
    }

    if (depthReading > config.pitDepth) {
        depthReading = config.pitDepth;
    }

    Serial.print(F("DEBUG: Depth acutal = "));
    Serial.println(depthReading);
    Serial.print(F("DEBUG: Pit depth = "));
    Serial.println(config.pitDepth);

    // To compute how *full* the pit is, we need to take the inverse of
    // the reading. So if it's 30 in deep, then it's 0% full (no water).
    // But if it's 0 in deep then the water is literally right under the
    // sensor, which at that point would mean it's 100% full and likely
    // flooding. So the depth reading is the actual pit depth. We need
    // water depth, so we subtract the current reading from the known
    // pit depth (+/- a potential offset for sensor position).

    if (depthReading == 0) {
        percentFull = 100;
    }
    else {
        percentFull = sensor.getPercentFull();
    }

    if (percentFull < 0) {
        percentFull = 0;
    }

    lastDepth = depthReading;
    Serial.print(F("INFO: Water level change event. Depth: "));
    Serial.print(depthReading);
    Serial.print(F(" inches ("));
    Serial.print(percentFull);
    Serial.println(F("%)"));

    if (!manualPumpMode) {
        if (pumpRelay.isOpen() && percentFull >= config.pumpActivatePercent) {
            togglePump(true);
        }
        else if (pumpRelay.isClosed() && percentFull <= config.pumpDeactivatePercent) {
            togglePump(false);
        }
    }

    if (depthReading <= ALARM_DEPTH_INCHES) {
       if (alarmLED.isOff()) {
           alarmLED.on();
           if (!alarmDisabled) {
            alarmBuzzer.on();
           }
           Serial.println(F("WARN: Water level CRITICAL!"));
       }
    }
    else {
        if (alarmLED.isOn()) {
            alarmLED.off();
            alarmBuzzer.off();
            Serial.println(F("INFO: Water level within safe limit."));
        }
    }

    delay(100);
    publishSystemState();
}

/**
 * Initializes the crash monitor and dump any previous crash data to the serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * Callback handler for the 'activate pump' command. This activates the pump
 * relay and waits for the user to press 'RETURN' to stop the pump
 */
void handleActivatePump() {
    // Manually activate pump.
    togglePump(true);
    delay(500);
    Serial.println(F("--- Press RETURN to stop pump ---"));
    while (Serial.available()) {
        ESPCrashMonitor.iAmAlive();
        char c = Serial.read();
        if (c == '\r') {
            togglePump(false);
            break;
        }
    }

    Serial.println();
}

/**
 * Callback handler for the 'factory restore' command. This restores the
 * configuration back to factory defaults and reboots the device.
 */
void handleFactoryRestore() {
    doFactoryRestore();
}

/**
 * Callback handler for the 'configure host name' command. This sets the
 * new host name in the running configuration and reinitializes MDNS.
 */
void onNewHostName(const char* newHostName) {
    config.hostname = newHostName;
    initMDNS();
}

/**
 * Callback handler for the 'configure network' command when switching to DHCP
 * mode. This switches from static network config to DHCP (if not already set).
 */
void onSwitchToDhcp() {
    if (config.useDhcp) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        config.useDhcp = true;
        Serial.println(F("INFO: Set DHCP mode."));
        WiFi.config(0U, 0U, 0U, 0U);
    }
}

/**
 * Callback handler for the 'configure network' command when switching to a
 * static network config. This configures the network interface with the
 * new static address settings.
 */
void onSwitchToStaticConfig(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    config.ip = newIp;
    config.sm = newSm;
    config.gw = newGw;
    config.dns = newDns;
    WiFi.config(config.ip, config.gw, config.sm, config.dns);  // If actual IP set, then disables DHCP and assumes static.
}

/**
 * Callback handler for the 'reconnect' command. This attempts to reconnect to
 * the configured WiFi network. If successful, resumes normal operation, dumps
 * the network config to console and exits the CLI. If unsuccessful, warns the
 * user and returns to the command menu. 
 */
void onReconnectFromConsole() {
    // Attempt to reconnect to WiFi.
    onCheckWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        printNetworkInfo();
        resumeNormal();
    }
    else {
        Serial.println(F("ERROR: Still no network connection."));
        Console.enterCommandInterpreter();
    }
}

/**
 * Callback handler for the 'configure WiFi' command. This sets the new
 * SSID and password in the running config and connects to the new WiFi
 * network.
 */
void onWifiConfig(String newSsid, String newPassword) {
    config.ssid = newSsid;
    config.password = newPassword;
    connectWifi();
}

/**
 * Saves the running configuration to flash memory so that the current
 * configuration settings can be used on startup. IMPORTANT NOTE:
 * If any changes to the current running configuration are NOT saved,
 * then they will be discarded upon reboot and reverted to their last
 * known (or default) values.
 */
void onSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWiFi();
}

/**
 * Callback handler that receives the new MQTT configuration settings. This
 * disconnects from any currently connected MQTT broker, sets the new config
 * and then reinitializes the MQTT client.
 * @param newBroker The new MQTT broker hostname or address.
 * @param newPort The new MQTT host port.
 * @param newUsername The new MQTT username.
 * @param newPass The new MQTT password.
 * @param newConChan The new MQTT control channel to subscribe to.
 * @param newStatChan The new MQTT status chanel to publish to.
 */
void onMqttConfigCommand(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan) {
    if (mqttClient.connected()) {
        mqttClient.unsubscribe(config.mqttTopicControl.c_str());
        mqttClient.disconnect();
    }

    config.mqttBroker = newBroker;
    config.mqttPort = newPort;
    config.mqttUsername = newUsername;
    config.mqttPassword = newPass;
    config.mqttTopicControl = newConChan;
    config.mqttTopicStatus = newStatChan;
    initMQTT();
    Serial.println();
}

/**
 * Callback handler for the 'report pit depth' command. This averages a
 * multi-sample reading and then displays it in the console.
 */
void reportPitDepthHandler() {
    ESPCrashMonitor.defer();
    reportDepth();
    delay(1000);
}

/**
 * Callback handler that receives the new pit depth for the 'configure pit
 * depth' command. This sets the new pit depth in the running configuration.
 */
void configPitDepthHandler(int newDepth) {
    config.pitDepth = newDepth;
    Serial.print(F("INFO: New depth = "));
    Serial.println(config.pitDepth);
    Serial.println();
    sensor.setTapeLengthCm(config.pitDepth / 2.54);
}

/**
 * Callback handler for the 'disable alarm' command. This turns off the
 * alarm buzzer (if on), then sets the disabled flag to prevent the system
 * from further activating it.
 */
void handleAlarmDisabled() {
    alarmBuzzer.off();
    alarmDisabled = true;
    Serial.println(F("WARN: Alarm disabled."));
    Serial.println();
}

/**
 * Callback handler for the 'enable alarm' command. This clears the disable
 * flag allowing the system to activate the alarm buzzer whenever the alarm
 * condition is met again.
 */
void handleAlarmEnabled() {
    alarmDisabled = false;
    Serial.println(F("INFO: Alarm enabled."));
    Serial.println();
}

/**
 * Handler for when the user gets a minimum level reading
 * during calibration.
 */
void handleMinSensorLevel() {
    float reading = sensor.read();
    Serial.print(F("INFO: Min Sensor value: "));
    Serial.println(reading);
    sensor.setMinValue(reading);
}

/**
 * Handler for when the user ges maximum level reading
 * during calibration.
 */
void handleMaxSensorLevel() {
    float reading = sensor.read();
    Serial.print(F("INFO: Max Sensor value: "));
    Serial.println(reading);
    sensor.setMaxValue(reading);
}

/**
 * Initialize the CLI.
 */
void initConsole() {
    Serial.print(F("INIT: Initializing console... "));
    Console.setHostName(config.hostname);
    Console.setMqttConfig(
        config.mqttBroker,
        config.mqttPort,
        config.mqttUsername,
        config.mqttPassword,
        config.mqttTopicControl,
        config.mqttTopicStatus
    );

    Console.onActivatePump(handleActivatePump);
    Console.onRebootCommand(reboot);
    Console.onScanNetworks(getAvailableNetworks);
    Console.onHostNameChange(onNewHostName);
    Console.onDhcpConfig(onSwitchToDhcp);
    Console.onStaticConfig(onSwitchToStaticConfig);
    Console.onReconnectCommand(onReconnectFromConsole);
    Console.onWiFiConfigCommand(onWifiConfig);
    Console.onResumeCommand(resumeNormal);
    Console.onGetNetInfoCommand(printNetworkInfo);
    Console.onSaveConfigCommand(onSaveConfig);
    Console.onMqttConfigCommand(onMqttConfigCommand);
    Console.onConsoleInterrupt(failSafe);
    Console.onFactoryRestore(handleFactoryRestore);
    Console.onReportPitDepth(reportPitDepthHandler);
    Console.onConfigurePithDepth(configPitDepthHandler);
    Console.onRunDiagnostics(runDiagnostics);
    Console.onAlarmDisabled(handleAlarmDisabled);
    Console.onAlarmDisabled(handleAlarmEnabled);
    Console.onMinSensorReading(handleMinSensorLevel);
    Console.onMaxSensorReading(handleMaxSensorLevel);

    Serial.println(F("DONE"));
}

/**
 * Bootstrap routine. Executes once at boot and initializes all subsystems in sequence.
 */
void setup() {
    // This boot sequence is important. DO NOT ALTER.
    initSerial();
    initCrashMonitor();
    initOutputs();
    initDepthSensor();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    initMQTT();
    initTaskManager();
    initConsole();
    Serial.println(F("INIT: Boot sequence complete."));
    pumpLED.off();
    alarmLED.off();
    sysState = SystemState::NORMAL;
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * Main loop. Executes all tasks that need to be, handles incoming web server requests,
 * and OTA update requests.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    Console.checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}