#ifndef TelemetryHelper_h
#define TelemetryHelper_h

#include <Arduino.h>

/**
 * Possible system states.
 */
enum class SystemState: uint8_t {
    /**
     * The system is currently running the boot sequence.
     */
    BOOTING = 0,

    /**
     * The system is operating normally.
     */
    NORMAL = 1,

    /**
     * The system is currently performing an OTA update.
     */
    UPDATING = 2,

    /**
     * The system is disabled. Either intentionally by the user or
     * by the system due to some internal system condition such as
     * falling into failsafe/configuration mode.
     */
    DISABLED = 3
};

/**
 * Possible system control commands.
 */
enum class ControlCommand: uint8_t {
    /**
     * Disables the system, preventing control commands.
     */
    DISABLE = 0,

    /**
     * Enables the system if currently disabled.
     */
    ENABLE = 1,

    /**
     * Activates the pump.
     */
    ACTIVATE = 2,

    /**
     * Deactivate the pump.
     */
    DEACTIVATE = 3,

    /**
     * Reboot the system.
     */
    REBOOT = 4,

    /**
     * Request system status. Causes a status message to be published to
     * the MQTT status topic.
     */
    REQUEST_STATUS = 5,

    /**
     * Disable (silence) the audible alarm. Does not disable the alarm LED or
     * prevent push notifications from openhab.
     */
    DISABLE_ALARM = 6,

    /**
     * Enable the audible alarm.
     */
    ENABLE_ALARM = 7
};

/**
 * Helper class providing static telemetry helper methods.
 */
class TelemetryHelper
{
public:
    /**
     * Gets a string describing the specified MQTT client state.
     * @param state The state to get the description for.
     * @return A string describing the MQTT state.
     */
    static String getMqttStateDesc(int state);
};

#endif