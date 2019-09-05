#!/bin/bash

# This tool will completely clear the flash memory on your ESP8266. This is useful
# for troubleshooting flash memory corruption or clearing out old firmware and
# stored data. Requires esptool.py. If not installed, you will be prompted to
# install it.

FLASHER="$( which esptool.py )"
if [[ -z "${FLASHER}" ]]; then
    echo "WARN: Flash tool not installed."
    read -rp $'Do you want to install now? (Y/n): ' choice
    if [[ "${choice}" = "y" || "${choice}" = "Y" ]]; then
        pip install esptool
        FLASHER="$( which esptool.py )"
        if [[ -z "${FLASHER}" ]]; then
            echo "ERROR: Installation failed."
            exit 1
        fi
    else
        exit 0
    fi
fi

# Modify this line to reflect the path to your USB-to-Serial device.
${FLASHER} -p /dev/cu.usbserial-AL05HSL2 erase_flash