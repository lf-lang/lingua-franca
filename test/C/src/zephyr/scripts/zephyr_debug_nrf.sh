# This is also assumed to be run inside build/zephyr folder
echo "--- Executing zephyr_debug_nrf.sh"

NRF_GDB=~/bin/zephyr-sdk-0.15.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb

eval $NRF_GDB --command $LF_SRC_DIRECTORY/../scripts/zephyr_debug_nrf.gdb
