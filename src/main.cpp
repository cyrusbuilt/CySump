/**
 * CySump
 * v1.1
 * 
 * Author:
 *  Cyrus Brunner <cyrusbuilt at gmail dot com>
 */
#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <FS.h>
#include <time.h>
#include "Buzzer.h"
#include "LED.h"
#include "Relay.h"
#include "HCSR04.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "ArduinoJson.h"
#include "PubSubClient.h"
#include "TelemetryHelper.h"
#include "ESP8266Ping.h"
#include "config.h"

#define FIRMWARE_VERSION "1.1"

// Workaround to allow an MQTT packet size greater than the default of 128.
#ifdef MQTT_MAX_PACKET_SIZE
#undef MQTT_MAX_PACKET_SIZE
#endif
#define MQTT_MAX_PACKET_SIZE 200

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
void onDepthChange(HCSR04Info* sender);

// Global vars
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
HCSR04 depthSensor(PIN_TRIGGER, PIN_ECHO, onDepthChange);
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
volatile int percentFull = 0;
bool isDHCP = false;
bool filesystemMounted = false;
bool connSecured = false;
volatile SystemState sysState = SystemState::BOOTING;
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
 * Gets an IPAddress value from the specified string.
 * @param value The string containing the IP.
 * @return The IP address.
 */
IPAddress getIPFromString(String value) {
    unsigned int ip[4];
    unsigned char buf[value.length()];
    value.getBytes(buf, value.length());
    const char* ipBuf = (const char*)buf;
    sscanf(ipBuf, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

/**
 * Get and print the pump status.
 * @return The pump status ("OFF" or "ON")
 */
String pumpStatus() {
    String state = "ON";
    if (pumpRelay.isClosed()) {
        state = "OFF";
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
    else if (percentFull < 50) {
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

        DynamicJsonDocument doc(200);
        doc["client_id"] = hostName;
        doc["pumpState"] = pumpStatus();
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["waterLevelPercent"] = percentFull;
        doc["pitState"] = pitState(percentFull);

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
 * Handles the water depth change event. This checks to see how much
 * water is in the sump pit, and if there is too much water, triggers
 * an alarm audible alarm. Also publishes the system state to MQTT.
 * @param sender The sensor info.
 */
void onDepthChange(HCSR04Info* sender) {
    // TODO Probably need some kind of calibration routine. The distance
    // measured is the distance from wherever the sensor is in space
    // and the bottom of the pit. If the sensor is mounted far enough
    // above the top of the pit, this could skew things. Likewise, if
    // the sensor is positioned slightly inside the top of the pit, we
    // need to account for that.
    double depth = sender->currentDistance * CM_TO_IN_MULTIPLIER;

    // To compute how *full* the pit is, we need to take the inverse of
    // the reading. So if it's 30 in deep, then it's 0% full (no water).
    // But if it's 0 in deep then the water is literally right under the
    // sensor, which at that point would mean it's 100% full and likely
    // flooding. So the depth reading is the actual pit depth. We need
    // water depth, so we subtract the current reading from the known
    // pit depth (+/- a potential offset for sensor position).
    depth = PIT_DEPTH_INCHES - depth;
    if (depth < 0) {
        depth = 0;
    }

    if (depth == 0) {
        percentFull = 100;
    }
    else {
        percentFull = round((depth * 100) / PIT_DEPTH_INCHES);
    }

    Serial.print(F("INFO: Water level change event. Depth: "));
    Serial.print(depth);
    Serial.print(F(" inches ("));
    Serial.print(percentFull);
    Serial.print(F("%)"));

    // TODO make these threshholds constant (and possibly configurable).
    if (pumpRelay.isOpen() && percentFull >= 40) {
        togglePump(true);
    }
    else if (pumpRelay.isClosed() && percentFull <= 5) {
        togglePump(false);
    }

    if (sender->isAlarm) {
       if (alarmLED.isOff()) {
           alarmLED.on();
           alarmBuzzer.on();
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

    publishSystemState();
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
 * Waits for user input from the serial console.
 */
void waitForUserInput() {
    while (Serial.available() < 1) {
        ESPCrashMonitor.iAmAlive();
        delay(50);
    }
}

/**
 * Gets string input from the serial console.
 * @param isPassword If true, echos back a '*' instead of the character
 * that was entered.
 * @return The string that was entered.
 */
String getInputString(bool isPassword = false) {
    char c;
    String result = "";
    bool gotEndMarker = false;
    while (!gotEndMarker) {
        ESPCrashMonitor.iAmAlive();
        if (Serial.available() > 0) {
            c = Serial.read();
            if (c == '\n') {
                gotEndMarker = true;
                break;
            }

            if (isPassword) {
                Serial.print('*');
            }
            else {
                Serial.print(c);
            }
            result += c;
        }
    }

    return result;
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
    doc["serverFingerPrintPath"] = serverFingerprintPath;
    doc["caCertificatePath"] = caCertificatePath;
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
    serverFingerprintPath = doc["serverFingerprintPath"].as<String>();
    caCertificatePath = doc["caCertificatePath"].as<String>();
    #ifdef ENABLE_OTA
        otaPort = doc["otaPort"].as<int>();
        otaPassword = doc["otaPassword"].as<String>();
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

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

    wifiClient.setTrustAnchors(&caCertX509);
    wifiClient.allowSelfSignedCerts();

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
        wifiClient.setFingerprint(fingerprintString.c_str());
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

/**
 * Confirms with the user that they wish to do a factory restore. If so, then
 * clears the current configuration file in SPIFFS, then reboots. Upon reboot,
 * a new config file will be generated with default values.
 */
void doFactoryRestore() {
    Serial.println();
    Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)?"));
    waitForUserInput();
    String str = getInputString();
    if (str == "Y" || str == "y") {
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
        if (!connSecured) {
            connSecured = verifyTLS();
            if (!connSecured) {
                Serial.println(F("ERROR: Unable to establish TLS connection to host."));
                Serial.println(F("ERROR: Invalid certificate or SSL negotiation failed."));
                return false;
            }
        }

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
            togglePump(true);
            break;
        case ControlCommand::DEACTIVATE:
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
 * Prompts the user with a configuration screen and waits for
 * user input.
 */
void promptConfig() {
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F("= Command menu:              ="));
    Serial.println(F("=                            ="));
    Serial.println(F("= r: Reboot                  ="));
    Serial.println(F("= c: Configure network       ="));
    Serial.println(F("= m: Configure MQTT settings ="));
    Serial.println(F("= s: Scan wireless networks  ="));
    Serial.println(F("= n: Connect to new network  ="));
    Serial.println(F("= w: Reconnect to WiFi       ="));
    Serial.println(F("= e: Resume normal operation ="));
    Serial.println(F("= g: Get network info        ="));
    Serial.println(F("= a: Activate pump           ="));
    Serial.println(F("= x: Get pit depth           ="));
    Serial.println(F("= o: Run diagnostics         ="));
    Serial.println(F("= f: Save config changes     ="));
    Serial.println(F("= z: Restore default config  ="));
    Serial.println(F("=                            ="));
    Serial.println(F("=============================="));
    Serial.println();
    Serial.println(F("Enter command choice (r/c/s/n/w/e/g): "));
    waitForUserInput();
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
 * Prompts the user for (and the applies) new MQTT configuration settings.
 */
void configureMQTT() {
    mqttClient.unsubscribe(controlChannel.c_str());
    mqttClient.disconnect();

    Serial.print(F("Current MQTT broker = "));
    Serial.println(mqttBroker);
    Serial.println(F("Enter MQTT broker address:"));
    waitForUserInput();
    mqttBroker = getInputString();
    Serial.println();
    Serial.print(F("New broker = "));
    Serial.println(mqttBroker);

    Serial.print(F("Current port = "));
    Serial.println(mqttPort);
    Serial.println(F("Enter MQTT broker port:"));
    waitForUserInput();
    String str = getInputString();
    mqttPort = str.toInt();
    Serial.println();
    Serial.print(F("New port = "));
    Serial.println(mqttPort);

    Serial.print(F("Current control channel = "));
    Serial.println(controlChannel);
    Serial.println(F("Enter MQTT control channel:"));
    waitForUserInput();
    controlChannel = getInputString();
    Serial.println();
    Serial.print(F("New control channel = "));
    Serial.println(controlChannel);

    Serial.print(F("Current status channel = "));
    Serial.println(statusChannel);
    Serial.println(F("Enter MQTT status channel:"));
    waitForUserInput();
    statusChannel = getInputString();
    Serial.println();
    Serial.print(F("New status channel = "));
    Serial.println(statusChannel);

    Serial.print(F("Current username: "));
    Serial.println(mqttUsername);
    Serial.println(F("Enter new username, or just press enter to clear:"));
    waitForUserInput();
    mqttPassword = getInputString();
    Serial.print(F("New MQTT username = "));
    Serial.println(mqttUsername);

    Serial.print(F("Current password: "));
    for (uint8_t i = 0; i < mqttPassword.length(); i++) {
        Serial.print(F("*"));
    }

    Serial.println();
    Serial.print(F("Enter new password, or just press enter to clear"));
    waitForUserInput();
    mqttPassword = getInputString(true);

    initMQTT();
    Serial.println();
}

/**
 * Prompts the user for, and configures static IP settings.
 */
void configureStaticIP() {
    isDHCP = false;
    Serial.println(F("Enter IP address: "));
    waitForUserInput();
    ip = getIPFromString(getInputString());
    Serial.print(F("New IP: "));
    Serial.println(ip);

    Serial.println(F("Enter gateway: "));
    waitForUserInput();
    gw = getIPFromString(getInputString());
    Serial.print(F("New gateway: "));
    Serial.println(gw);

    Serial.println(F("Enter subnet mask: "));
    waitForUserInput();
    sm = getIPFromString(getInputString());
    Serial.print(F("New subnet mask: "));
    Serial.println(sm);

    Serial.println(F("Enter DNS server: "));
    waitForUserInput();
    dns = getIPFromString(getInputString());
    Serial.print(F("New DNS server: "));
    Serial.println(dns);

    WiFi.config(ip, gw, sm, dns);  // If actual IP set, then disables DHCP and assumes static.
}

/**
 * Prompts the user for and then attempts to connect to a new
 * WiFi network.
 */
void configureWiFiNetwork() {
    Serial.println(F("Enter new SSID: "));
    waitForUserInput();
    ssid = getInputString();
    Serial.print(F("SSID = "));
    Serial.println(ssid);

    Serial.println(F("Enter new password: "));
    waitForUserInput();
    password = getInputString();
    Serial.print(F("Password = "));
    Serial.println(password);

    connectWifi();
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
    double depth = depthSensor.measureDistanceCm();
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

    Serial.println(F("DIAG: Verifying TLS connection... "));
    if (verifyTLS()) {
        Serial.println(F("DIAG: TLS verification PASSED"));
    }
    else {
        Serial.println(F("DIAG: TLS verification FAIL"));
    }

    Serial.println(F("INFO: Self-diagnostics complete."));
}

/**
 * Simply report the pit depth in inches.
 */
void reportDepth() {
    Serial.print(F("INFO: Pit depth = "));
    Serial.print(depthSensor.measureDistanceIn());
    Serial.println(F(" inches."));
}

/**
 * Checks commands entered by the user via serial input and carries out
 * the specified action if valid.
 */
void checkCommand() {
    String str = "";
    char incomingByte = Serial.read();
    switch (incomingByte) {
        case 'r':
            // Reset the controller.
            reboot();
            break;
        case 's':
            // Scan for available networks.
            getAvailableNetworks();
            promptConfig();
            checkCommand();
            break;
        case 'c':
            // Set hostname.
            Serial.print(F("Current host name: "));
            Serial.println(hostName);
            Serial.println(F("Set new host name: "));
            waitForUserInput();
            hostName = getInputString();
            initMDNS();

            // Change network mode.
            Serial.println(F("Choose network mode (d = DHCP, t = Static):"));
            waitForUserInput();
            checkCommand();
            break;
        case 'd':
            // Switch to DHCP mode.
            if (isDHCP) {
                Serial.println(F("INFO: DHCP mode already set. Skipping..."));
                Serial.println();
            }
            else {
                isDHCP = true;
                Serial.println(F("INFO: Set DHCP mode."));
                WiFi.config(0U, 0U, 0U, 0U);
            }
            promptConfig();
            checkCommand();
            break;
        case 't':
            // Switch to static IP mode. Request IP settings.
            configureStaticIP();
            promptConfig();
            checkCommand();
            break;
        case 'w':
            // Attempt to reconnect to WiFi.
            onCheckWiFi();
            if (WiFi.status() == WL_CONNECTED) {
                printNetworkInfo();
                resumeNormal();
            }
            else {
                Serial.println(F("ERROR: Still no network connection."));
                promptConfig();
                checkCommand();
            }
            break;
        case 'n':
            // Connect to a new wifi network.
            configureWiFiNetwork();
            promptConfig();
            checkCommand();
            break;
        case 'e':
            // Resume normal operation.
            resumeNormal();
            break;
        case 'g':
            // Get network info.
            printNetworkInfo();
            promptConfig();
            checkCommand();
            break;
        case 'a':
            togglePump(true);
            delay(500);
            Serial.println(F("--- Press ENTER to stop pump ---"));
            while (Serial.available()) {
                ESPCrashMonitor.iAmAlive();
                char c = Serial.read();
                if (c == '\r') {
                    togglePump(false);
                    break;
                }
            }
            promptConfig();
            checkCommand();
            break;
        case 'x':
            reportDepth();
            delay(2000);
            promptConfig();
            checkCommand();
            break;
        case 'f':
            // Save configuration changes and restart services.
            saveConfiguration();
            WiFi.disconnect(true);
            onCheckWiFi();
            promptConfig();
            checkCommand();
            break;
        case 'z':
            // Reset config to factory default.
            doFactoryRestore();
            promptConfig();
            checkCommand();
            break;
        case 'm':
            // Set MQTT settings.
            configureMQTT();
            promptConfig();
            checkCommand();
            break;
        case 'o':
            // Run hardware diagnostics.
            runDiagnostics();
            promptConfig();
            checkCommand();
            break;
        default:
            // Specified command is invalid.
            Serial.println(F("WARN: Unrecognized command."));
            promptConfig();
            checkCommand();
            break;
    }
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
    promptConfig();
    checkCommand();
}

/**
 * Check for the user to press the 'i' key at the serial console to
 * interrupt normal operation and present the configuration menu.
 */
void checkInterrupt() {
    if (Serial.available() > 0 && Serial.read() == 'i') {
        failSafe();
    }
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
 * Initialize sensor inputs.
 */
void initSensors() {
    Serial.print(F("INIT: Initializing sensors... "));
    depthSensor.begin();
    depthSensor.setAlarmDistance(ALARM_DEPTH_INCHES);
    Serial.println(F("DONE"));
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
    depthSensor.loop();
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
 * Bootstrap routine. Executes once at boot and initializes all subsystems in sequence.
 */
void setup() {
    // This boot sequence is important. DO NOT ALTER.
    initSerial();
    initCrashMonitor();
    initOutputs();
    initSensors();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    
    if (loadCertificates()) {
        if (verifyTLS()) {
            connSecured = true;
            initMQTT();
        }
    }
    
    initTaskManager();
    Serial.println(F("INIT: Boot sequence complete."));
    sysState = SystemState::NORMAL;
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * Main loop. Executes all tasks that need to be, handles incoming web server requests,
 * and OTA update requests.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}