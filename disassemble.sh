#!/usr/bin/env bash

# When disassembly is complete, open the text file and it should have a line
# that corresponds with the byte-address in the crash report that gets reported
# by the firmware on boot. For example: If you have a byte-address of 0x0730,
# then search the text file for a line that starts with '730'. That is the
# line of code that the firmware crashed at.

# Get firmware path. If it exists, decompile and dump it to a text file.
echo
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
BOARD=$(awk -F "=" '/board/ {print $2}' ${SCRIPT_DIR}/platformio.ini | tr -d ' ')
FIRMWARE=${SCRIPT_DIR}/.pio/build/${BOARD}/firmware.elf
DUMPFILE=${SCRIPT_DIR}/disassembly.txt

function disassemble {
    echo "INFO: Disassembling firmware ..."
    ${DUMPER} -SC -d ${FIRMWARE} > ${DUMPFILE}
    if [ $? -eq 0 ]; then
        echo
        echo "INFO: Disassembly succeeded."
        echo "INFO: Output location: ${DUMPFILE}"
        echo
        exit 0
    else
        echo
        echo "ERROR: Disassembly failed."
    fi
}

function buildFirmware {
    echo
    echo "INFO: Checking for platformio ..."
    local compiler="$( which platformio )"
    if [[ -z "${compiler}" ]]; then
        echo
        echo "ERROR: platformio not found. You may need to install the shell commands."
        echo
        exit 1
    fi

    # Make sure we run from the project directory.
    cd ${SCRIPT_DIR}
    platformio run
}

echo "INFO: Checking for ${FIRMWARE} ..."
if [ -f $FIRMWARE ]; then
    echo
    echo "INFO: Found firmware. Checking for disassembler ..."
    if [ -f ${DUMPFILE} ]; then
        rm -f ${DUMPFILE}
    fi

    # Look for the disassembler in platformio's usual location.
    DUMPER=~/.platformio/packages/toolchain-xtensa/bin/xtensa-lx106-elf-objdump
    if [ ! -f ${DUMPER} ]; then
        echo "ERROR: Disassembler not found. Make sure the Arduino software is installed."
        exit 1
    fi

    disassemble
else
    echo
    echo
    choice=""
    echo "WARN: Firmware not found."
    read -rp $'Would you like to run a build now?\n[Y/n] ' choice
    if [[ "${choice}" = "y" ]] || [[ "${choice}" = "Y" ]]; then
        buildFirmware
        disassemble
    else
        echo
        echo "User cancelled."
    fi
fi

exit 1