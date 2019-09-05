/**
 * 
 */

#ifndef config_h
#define config_h

#include <IPAddress.h>

// Configuration
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define ENABLE_MDNS                             // Comment this line to disable the MDNS.
#define PIT_DEPTH_INCHES 30                     // The depth of the sump pit (inches).
#define ALARM_DEPTH_INCHES 5                    // The depth at which to consider an alarm state (inches left until full)
#define CONFIG_FILE_PATH "/config.json"         // The config file path. Do not alter unless you are sure.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_password_here"   // Put your WiFi password here.
#define CLOCK_TIMEZONE -4                       // The timezone this device is located in. (For example, EST when observing DST = GMT-4, when not = GMT-5)
#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CHECK_SENSORS_INTERVAL 2000             // How often to check sensors (milliseconds).
#define CLOCK_SYNC_INTERVAL 3600000             // How often to sync the local clock with NTP (milliseconds).
#define DEVICE_NAME "CYSUMP"                    // The device name.
#define CHECK_MQTT_INTERVAL 35000               // MQTT connectivity check interval (milliseconds).
#define MQTT_TOPIC_STATUS "cysump/status"       // MQTT status channel to publish to.
#define MQTT_TOPIC_CONTROL "cysump/control"     // MQTT control channel to subscribe to.
#define MQTT_BROKER "your_mqtt_broker_here"     // MQTT broker hostname or IP.
#define MQTT_PORT 8883                          // MQTT port number.
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_PASSWORD "your_ota_password_here"  // The OTA updater password.
#endif
IPAddress ip(192, 168, 0, 202);                 // The default static host IP.
IPAddress gw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress sm(255, 255, 255, 0);                 // The default static subnet mask.
IPAddress dns(gw);                              // The default static DNS server IP (same as gateway for most residential setups)

#endif