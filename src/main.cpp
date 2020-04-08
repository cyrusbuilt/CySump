/**
 * CySump
 * v1.4
 * 
 * Author:
 *  Cyrus Brunner <cyrusbuilt at gmail dot com>
 */

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif
#include <Arduino.h>
#include <ESP8266WiFi.h>
#ifdef ENABLE_TLS
    #include <WiFiClientSecure.h>
#else
    #include <WiFiClient.h>
#endif
#include <FS.h>
#include <time.h>
#include "Buzzer.h"
#include "LED.h"
#include "Relay.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "ArduinoJson.h"
#include "PubSubClient.h"
#include "TelemetryHelper.h"
#include "ESP8266Ping.h"
#include "NewPing.h"
#include "config.h"
#include "Console.h"

#define FIRMWARE_VERSION "1.4"

#define MAX_DISTANCE_CM 400       /** The maximum supported distance for reliable ranging (cm). */

// Pin definitions
#define PIN_WIFI_LED 2
#define PIN_ALARM_LED 4
#define PIN_PUMP_LED 5
#define PIN_TRIGGER 12
#define PIN_ECHO 14
#define PIN_RELAY 15
#define PIN_BUZZER 16

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
#ifdef ENABLE_TLS
    WiFiClientSecure wifiClient;
#else
    WiFiClient wifiClient;
#endif
PubSubClient mqttClient(wifiClient);
NewPing depthSensor(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE_CM);
Relay pumpRelay(PIN_RELAY, onRelayStateChange, "GARAGE_DOOR");
LED alarmLED(PIN_ALARM_LED, NULL);
LED wifiLED(PIN_WIFI_LED, NULL);
LED pumpLED(PIN_PUMP_LED, NULL);
Buzzer alarmBuzzer(PIN_BUZZER, NULL, "AlarmBuzzer");
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tSyncClock(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Task tCheckSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onCheckSensors);
Scheduler taskMan;
String hostName = DEVICE_NAME;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;
String mqttBroker = MQTT_BROKER;
String controlChannel = MQTT_TOPIC_CONTROL;
String statusChannel = MQTT_TOPIC_STATUS;
String serverFingerprintPath;
String caCertificatePath;
String fingerprintString;
String mqttUsername = "";
String mqttPassword = "";
int mqttPort = MQTT_PORT;
unsigned long pitDepth = PIT_DEPTH_INCHES;
volatile int percentFull = 0;
volatile unsigned long lastDepth = 0;
bool isDHCP = false;
bool filesystemMounted = false;
bool manualPumpMode = false;
#ifdef ENABLE_TLS
    bool connSecured = false;
#endif
volatile SystemState sysState = SystemState::BOOTING;
bool alarmDisabled = false;
#ifdef ENABLE_OTA
    int otaPort = OTA_HOST_PORT;
    String otaPassword = OTA_PASSWORD;
#endif

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

        DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
        doc["client_id"] = hostName;
        doc["pumpState"] = pumpStatus();
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["waterLevelPercent"] = percentFull;
        doc["pitState"] = pitState(percentFull);
        doc["waterDepth"] = lastDepth;
        doc["alarmEnabled"] = alarmDisabled ? "OFF" : "ON";

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(statusChannel.c_str(), jsonStr.c_str(), len)) {
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
    doc["hostname"] = hostName;
    doc["useDHCP"] = isDHCP;
    doc["ip"] = ip.toString();
    doc["gateway"] = gw.toString();
    doc["subnetMask"] = sm.toString();
    doc["dnsServer"] = dns.toString();
    doc["wifiSSID"] = ssid;
    doc["wifiPassword"] = password;
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    doc["mqttControlChannel"] = controlChannel;
    doc["mqttStatusChannel"] = statusChannel;
    doc["mqttUsername"] = mqttUsername;
    doc["mqttPassword"] = mqttPassword;
    #ifdef ENABLE_TLS
        doc["serverFingerPrintPath"] = serverFingerprintPath;
        doc["caCertificatePath"] = caCertificatePath;
    #endif
    doc["pitDepth"] = pitDepth;
    #ifdef ENABLE_OTA
        doc["otaPort"] = otaPort;
        doc["otaPassword"] = otaPassword;
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
    if (size > 1024) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Config file size is too large. Using default config."));
        configFile.close();
        return;
    }

    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);
    configFile.close();

    StaticJsonDocument<350> doc;
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        return;
    }

    hostName = doc["hostname"].as<String>();
    isDHCP = doc["useDHCP"].as<bool>();
    if (!ip.fromString(doc["ip"].as<String>())) {
        Serial.println(F("WARN: Invalid IP in configuration. Falling back to factory default."));
    }
    
    if (!gw.fromString(doc["gateway"].as<String>())) {
        Serial.println(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
    }
    
    if (!sm.fromString(doc["subnetMask"].as<String>())) {
        Serial.println(F("WARN: Invalid subnet mask in configuration. Falling back to default."));
    }

    if (!dns.fromString(doc["dnsServer"].as<String>())) {
        Serial.println(F("WARN: Invalid DSN server in configuration. Falling back to default."));
    }
    
    ssid = doc["wifiSSID"].as<String>();
    password = doc["wifiPassword"].as<String>();
    mqttBroker = doc["mqttBroker"].as<String>();
    mqttPort = doc["mqttPort"].as<int>();
    controlChannel = doc["mqttControlChannel"].as<String>();
    statusChannel = doc["mqttStatusChannel"].as<String>();
    mqttUsername = doc["mqttUsername"].as<String>();
    mqttPassword = doc["mqttPassword"].as<String>();
    #ifdef ENABLE_TLS
        serverFingerprintPath = doc["serverFingerprintPath"].as<String>();
        caCertificatePath = doc["caCertificatePath"].as<String>();
    #endif
    pitDepth = doc["pitDepth"].as<int>();
    #ifdef ENABLE_OTA
        otaPort = doc["otaPort"].as<int>();
        otaPassword = doc["otaPassword"].as<String>();
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

#ifdef ENABLE_TLS
/**
 * Loads the SSL certificates and server fingerprint necessary to establish
 * a connection the the MQTT broker over TLS.
 * @return true if the certificates and server fingerprint were successfully
 * loaded; Otherwise, false.
 */
bool loadCertificates() {
    Serial.print(F("INFO: Loading SSL certificates... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return false;
    }

    if (!SPIFFS.exists(caCertificatePath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: CA certificate does not exist."));
        return false;
    }

    File ca = SPIFFS.open(caCertificatePath, "r");
    if (!ca) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open CA certificate."));
        return false;
    }

    String caContents = ca.readString();
    ca.close();
    X509List caCertX509(caContents.c_str());

    wifiClient.allowSelfSignedCerts();
    wifiClient.setTrustAnchors(&caCertX509);

    if (!SPIFFS.exists(serverFingerprintPath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Server fingerprint file path does not exist."));
        return false;
    }

    File fp = SPIFFS.open(serverFingerprintPath, "r");
    if (!fp) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open fingerprint file."));
        return false;
    }

    String val;
    if (fp.available()) {
        String fileContent = fp.readString();
        val = fileContent.substring(fileContent.lastIndexOf("=") + 1);
        val.replace(':', ' ');
    }

    fp.close();
    if (val.length() > 0) {
        fingerprintString = val;
        fingerprintString.trim();
        if (!wifiClient.setFingerprint(fingerprintString.c_str())) {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Invalid fingerprint."));
            return false;
        }
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to read server fingerprint."));
        return false;
    }
    
    Serial.println(F("DONE"));
    return true;
}

/**
 * Verifies a connection can be made to the MQTT broker over TLS.
 * @return true if a connection to the MQTT broker over TLS was established
 * successfully; Otherwise, false.
 */
bool verifyTLS() {
    // Because it can take longer than expected to establish an
    // encrypted connection the MQTT broker, we need to disable
    // the watchdog to prevent reboot due to watchdog timeout during
    // connection, then re-enable when we are done.
    ESPCrashMonitor.disableWatchdog();

    // Currently, we sync the clock any time we need to verify TLS. This is
    // because in a future version, this will be required in order to validate
    // public CA certificates.
    onSyncClock();

    Serial.print(F("INFO: Verifying connectivity over TLS... "));
    //wifiClient.setX509Time(time(nullptr));
    bool result = wifiClient.connect(mqttBroker, mqttPort);
    if (result) {
        wifiClient.stop();
        Serial.println(F("DONE"));
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: TLS connection failed."));
    }

    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
    return result;
}
#endif

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
        Serial.print(mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(mqttPort);
        Serial.println(F("..."));
        #ifdef ENABLE_TLS
            if (!connSecured) {
                connSecured = verifyTLS();
                if (!connSecured) {
                    Serial.println(F("ERROR: Unable to establish TLS connection to host."));
                    Serial.println(F("ERROR: Invalid certificate or SSL negotiation failed."));
                    return false;
                }
            }
        #endif

        bool didConnect = false;
        if (mqttUsername.length() > 0 && mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(hostName.c_str(), mqttUsername.c_str(), mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(hostName.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(controlChannel);
            mqttClient.subscribe(controlChannel.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(statusChannel);
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
    if (!id.equals(hostName)) {
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

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

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
            if (!mdns.begin(hostName)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(hostName, "ota", otaPort);
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
    loadConfiguration();
}

/**
 * Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(mqttBroker.c_str(), mqttPort);
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
    if (isDHCP) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(ip, gw, sm, gw);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(ssid, password);
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
        if (Ping.ping(gw)) {
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
    unsigned long depth = depthSensor.ping_in();
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

    #ifdef ENABLE_TLS
        Serial.println(F("DIAG: Verifying TLS connection... "));
        if (verifyTLS()) {
            Serial.println(F("DIAG: TLS verification PASSED"));
        }
        else {
            Serial.println(F("DIAG: TLS verification FAIL"));
        }
    #endif

    Serial.println(F("INFO: Self-diagnostics complete."));
}

/**
 * Simply report the pit depth in inches.
 */
void reportDepth() {
    // Initialze results to 0.
    unsigned long results[5];
    for (unsigned long thisReading = 0; thisReading < sizeof(results); thisReading++) {
        results[thisReading] = 0;
    }

    // It takes ~100ms to measure distance, but we need to do some
    // smoothing here, so we'll get 5 samples and average them, which
    // should take ~650ms to complete (including delay).
    unsigned long total = 0;
    for (unsigned long thisReading = 0; thisReading < sizeof(results); thisReading++) {
        results[thisReading] = depthSensor.ping_in();
        total += results[thisReading];
        delay(30);
    }

    unsigned long average = total / 5;
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
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(ssid);
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
            ArduinoOTA.setPort(otaPort);
            ArduinoOTA.setHostname(hostName.c_str());
            ArduinoOTA.setPassword(otaPassword.c_str());
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

    // TODO Probably need some kind of calibration routine. The distance
    // measured is the distance from wherever the sensor is in space
    // and the bottom of the pit. If the sensor is mounted far enough
    // above the top of the pit, this could skew things. Likewise, if
    // the sensor is positioned slightly inside the top of the pit, we
    // need to account for that.
    unsigned long depth = depthSensor.ping_in();
    Serial.print(F("DEBUG: Depth acutal = "));
    Serial.println(depth);
    Serial.print(F("DEBUG: Pit depth = "));
    Serial.println(pitDepth);

    // To compute how *full* the pit is, we need to take the inverse of
    // the reading. So if it's 30 in deep, then it's 0% full (no water).
    // But if it's 0 in deep then the water is literally right under the
    // sensor, which at that point would mean it's 100% full and likely
    // flooding. So the depth reading is the actual pit depth. We need
    // water depth, so we subtract the current reading from the known
    // pit depth (+/- a potential offset for sensor position).
    //depth = pitDepth - depth;
    if (depth < 0) {
        depth = 0;
    }

    if (depth > pitDepth) {
        depth = pitDepth;
    }

    if (depth == 0) {
        percentFull = 100;
    }
    else {
        percentFull = 100 - ((depth * 100) / pitDepth);
    }

    if (percentFull < 0) {
        percentFull = 0;
    }

    lastDepth = depth;
    Serial.print(F("INFO: Water level change event. Depth: "));
    Serial.print(depth);
    Serial.print(F(" inches ("));
    Serial.print(percentFull);
    Serial.println(F("%)"));

    // TODO make these threshholds constant (and possibly configurable).
    if (!manualPumpMode) {
        if (pumpRelay.isOpen() && percentFull >= 70) {
            togglePump(true);
        }
        else if (pumpRelay.isClosed() && percentFull <= 15) {
            togglePump(false);
        }
    }

    if (depth <= ALARM_DEPTH_INCHES) {
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
    reboot();
}

/**
 * Callback handler for the 'configure host name' command. This sets the
 * new host name in the running configuration and reinitializes MDNS.
 */
void onNewHostName(const char* newHostName) {
    hostName = newHostName;
    initMDNS();
}

/**
 * Callback handler for the 'configure network' command when switching to DHCP
 * mode. This switches from static network config to DHCP (if not already set).
 */
void onSwitchToDhcp() {
    if (isDHCP) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        isDHCP = true;
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
    ip = newIp;
    sm = newSm;
    gw = newGw;
    dns = newDns;
    WiFi.config(ip, gw, sm, dns);  // If actual IP set, then disables DHCP and assumes static.
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
    ssid = newSsid;
    password = newPassword;
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
        mqttClient.unsubscribe(controlChannel.c_str());
        mqttClient.disconnect();
    }
    mqttBroker = newBroker;
    mqttPort = newPort;
    mqttUsername = newUsername;
    mqttPassword = newPass;
    controlChannel = newConChan;
    statusChannel = newStatChan;
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
    pitDepth = newDepth;
    Serial.print(F("INFO: New depth = "));
    Serial.println(pitDepth);
    Serial.println();
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
 * Initialize the CLI.
 */
void initConsole() {
    Serial.print(F("INIT: Initializing console... "));
    Console.setHostName(hostName);
    Console.setMqttConfig(mqttBroker, mqttPort, mqttUsername, mqttPassword, controlChannel, statusChannel);

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
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    
    #ifdef ENABLE_TLS
        if (loadCertificates() && verifyTLS()) {
            connSecured = true;
            initMQTT();
        }
    #else
        initMQTT();
    #endif
    
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