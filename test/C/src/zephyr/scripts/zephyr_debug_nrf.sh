# This is also assumed to be run inside build/zephyr folder
echo "--- Executing zephyr_debug_nrf.sh"

NRF_GDB=~/bin/zephyr-sdk-0.15.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb
ELF=$1


eval "$NRF_GDB -ex 'file $ELF' -ex 'target remote localhost:7777' -ex 'load' -ex 'mon reset 0' -ex 'b main' -ex 'c'"
