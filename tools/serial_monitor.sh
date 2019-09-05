#!/bin/bash

# This script simply starts the serial monitor on the target device.

# Modify this line to use the path of your USB-to-Serial adapter (if different).
platformio device monitor --baud 115200 --port /dev/cu.usbserial-AL05HSL2