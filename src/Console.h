#ifndef _CONSOLE_H
#define _CONSOLE_H

#include <Arduino.h>
#include <IPAddress.h>

/**
 * Provides a CLI and command menu for CySump.
 */
class ConsoleClass
{
public:
    /**
     * Default constructor.
     */
    ConsoleClass();

    /**
     * Converts a string representation of an IP into an IPAddress object.
     * @param value The IP address parsed from the specified string.
     */
    IPAddress getIPFromString(String value);

    /**
     * Waits for user input from the Serial console. Feeds the watchdog
     * while waiting to prevent reset.
     */
    void waitForUserInput();

    /**
     * Gets the string entered in the CLI.
     * @param isPassword Set true if the input string is expected to be a
     * password. If so, the input string echoed back will be masked with
     * asterisks. Default is false.
     * @return The string entered by the user.
     */
    String getInputString(bool isPassword = false);

    /**
     * Interrupts the current program and breaks into the CLI.
     */
    void enterCommandInterpreter();

    /**
     * Adds a reboot callback handler. This callback is executed when
     * the user selects the reboot command. It is assumed an actual
     * reboot will occur in the callback method, so it does not
     * attempt to return to the CLI after calling the handler.
     * @param rebootHandler The reboot callback method.
     */
    void onRebootCommand(void (*rebootHandler)());

    /**
     * Adds a scan networks callback handler.
     * @param scanHandler The callback method to execute when
     * the user selects the 'scan for networks' command.
     */
    void onScanNetworks(void (*scanHandler)());

    /**
     * Sets the default host name.
     * @param hostName The default host name.
     */
    void setHostName(String hostName);

    /**
     * Sets the MQTT configuration.
     * @param broker The MQTT broker host.
     * @param port The MQTT port.
     * @param username The MQTT username.
     * @param password The MQTT password.
     * @param conChan The control channel topic.
     * @param statChan The status channel topic.
     */
    void setMqttConfig(String broker, int port, String username, String password, String conChan, String statChan);

    /**
     * Sets the host name change callback handler.
     * @param hostNameChangeHandler The handler callback method that will execute
     * when the user changes the host name. The callback method will receive the
     * new host name entered by the user.
     */
    void onHostNameChange(void (*hostNameChangeHandler)(const char* newHostName));

    /**
     * Sets the callback method for when switching to DHCP.
     * @param dhcpHandler The callback method to execute if the user switches
     * the network config to DHCP mode.
     */
    void onDhcpConfig(void (*dhcpHandler)());

    /**
     * Sets the callback method to execute when the user switches to static IP.
     * @param staticHandler The callback method to receive the new static IP
     * network address settings including the new IP, subnet mask, gateway,
     * and DNS server addressses.
     */
    void onStaticConfig(void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns));

    /**
     * Sets the callback method to execute when the user selects the WiFi
     * reconnect command.
     * @param reconnectHandler The callback method to execute when reconnecting
     * to the WiFi network.
     */
    void onReconnectCommand(void (*reconnectHandler)());

    /**
     * Sets the callback method to execute when the user changes the WiFi
     * configuration settings.
     * @param wifiConfigHandler The callback method to receive the new SSID
     * and password entered by the user.
     */
    void onWiFiConfigCommand(void (*wifiConfigHandler)(String newSsid, String newPassword));

    /**
     * Sets the callback method to execute when the user selects the
     * 'resume normal operation' command which exits the CLI and resumes
     * normal tasks.
     * @param resumeHandler The callback method to execute when resuming normal
     * operation.
     */
    void onResumeCommand(void (*resumeHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'get network info' command.
     * @param netInfoHandler The callback method to execute for the
     * 'get network info' command.
     */
    void onGetNetInfoCommand(void (*netInfoHandler)());

    /**
     * Sets the callback method to execute when the user selects the 'save
     * configuration' command.
     * @param saveConfigHandler The callback method to execute when saving
     * config.
     */
    void onSaveConfigCommand(void (*saveConfigHandler)());

    /**
     * Sets the callback method to execute when the MQTT configuration changes.
     * @param mqttChangeHandler The callback method to receive the new MQTT
     * broker address, port, username, and password.
     */
    void onMqttConfigCommand(void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan));
    
    /**
     * Sets the callback method to execute when entering the CLI.
     * @param interruptHandler The callback method to execute.
     */
    void onConsoleInterrupt(void (*interruptHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'activate pump' command.
     * @param activatePumpHandler The callback method to execute.
     */
    void onActivatePump(void (*activatePumpHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'perform factory restore' command.
     * @param factoryRestoreHandler The callback method to execute
     * when performing a factory restore.
     */
    void onFactoryRestore(void (*factoryRestoreHandler)());

    /**
     * Sets the callback method to execute when the users selects the
     * 'report pit depth' command.
     * @param reportPitDepthHandler The callback method to execute
     * when performing a pit depth report.
     */
    void onReportPitDepth(void (*reportPitDepthHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'configure pit depth' command.
     * @param configPitDepthHandler The callback method that will receive
     * the new pit depth value.
     */
    void onConfigurePithDepth(void (*configPitDepthHandler)(int newDepth));

    /**
     * Sets the callback method to execute when the users selects the
     * 'run diagnostics' command.
     * @param runDiagsHandler The callback method to execute when
     * running diagnostics.
     */
    void onRunDiagnostics(void (*runDiagsHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'disable alarm' command.
     * @param onAlarmDisabled The callback method to execute when
     * disabling the alarm.
     */
    void onAlarmDisabled(void (*alarmDisabledHandler)());

    /**
     * Sets the callback method to execute when the user selects the
     * 'enable alarm' command.
     * @param alarmEnabledHandler The callback method to execute when
     * enabling the alarm.
     */
    void onAlarmEnabled(void (*alarmEnabledHnadler)());

    /**
     * Checks to see if the interrupt key ('i') has been pressed and fires
     * the interrupt handler if it has. This should be called from loop().
     */
    void checkInterrupt();

private:
    void displayMenu();
    void checkCommand();
    void configureStaticIP();
    void configureWiFiNetwork();
    void configMQTT();
    void configPitDepth();

    void (*rebootHandler)();
    void (*scanHandler)();
    void (*hostNameChangeHandler)(const char* newHostName);
    void (*dhcpHandler)();
    void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns);
    void (*reconnectHandler)();
    void (*wifiConfigHandler)(String newSsid, String newPassword);
    void (*resumeHandler)();
    void (*netInfoHandler)();
    void (*saveConfigHandler)();
    void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan);
    void (*interruptHandler)();
    void (*activatePumpHandler)();
    void (*factoryRestoreHandler)();
    void (*reportPitDepthHandler)();
    void (*configPitDepthHandler)(int newDepth);
    void (*runDiagsHandler)();
    void (*alarmDisabledHandler)();
    void (*alarmEnabledHandler)();
    String _hostName;
    String _mqttBroker;
    int _mqttPort;
    String _mqttUsername;
    String _mqttPassword;
    String _mqttControlChannel;
    String _mqttStatusChannel;
};

extern ConsoleClass Console;
#endif