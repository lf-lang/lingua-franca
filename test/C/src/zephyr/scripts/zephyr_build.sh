#!/usr/bin/bash

echo "Building Zephyr application"
echo "ZEPHYR_BASE=${ZEPHYR_BASE}"
SCRIPT_DIR=$(dirname $0)



set -e # Return on first error

# Verify command line arguments
if [ $# -lt 1 ]; then
    echo "ERROR: Please pass the board to zephyr_build.sh. e.g. ./zephyr_build.sh qemu_cortex_m3 flash"
    exit 1
fi

# Set some global variables
export BOARD=$1
export LF_SRC_DIRECTORY=$(pwd)

cp $LF_SRC_DIRECTORY/$SCRIPT_DIR/prj.conf $LF_SOURCE_GEN_DIRECTORY/
cp $LF_SRC_DIRECTORY/$SCRIPT_DIR/Kconfig $LF_SOURCE_GEN_DIRECTORY/

cd $LF_SOURCE_GEN_DIRECTORY

# Parse additional compile defs
COMPILE_DEFS=""
while IFS= read -r line; do
  COMPILE_DEFS="${COMPILE_DEFS} -D$line"
done < CompileDefinitions.txt
echo "Passing compile defs: $COMPILE_DEFS to cmake"

# Build project
west build -b $BOARD -- $COMPILE_DEFS

if [[ "$2" == "flash" ]]; then

if [[ "$BOARD" == "nrf"* ]]; then
    echo "--- Flashing to NRF board"
    # Flash application
    bash $LF_SRC_DIRECTORY/$SCRIPT_DIR/zephyr_flash_nrf.sh .

    # Debug application
    # FIXME: Fix the issues here. Why isnt gdb working when invoked from this script?
    # $LF_SRC_DIRECTORY/../scripts/zephyr_debug_nrf.sh
elif [[ "$BOARD" == "qemu"* ]]; then 
    echo "--- Executing on QEMU emulation"
    west build -t run
else
    echo "Unrecognized board $BOARD" 
    exit 1
fi

fi