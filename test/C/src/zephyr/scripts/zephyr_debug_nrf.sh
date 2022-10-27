# Takes build path as argument
echo "--- Executing zephyr_debug_nrf.sh"

SRC_GEN_PATH=$1

# FIXME: This is a hardcoded path to my installation of arm-zephyr-eabi-gdb
NRF_GDB=~/bin/zephyr-sdk-0.15.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb

ELF=$SRC_GEN_PATH/build/zephyr/zephyr.elf

# Start GDB and execute some startup commands to connect to target running on Debug chip.
eval "$NRF_GDB -ex 'file $ELF' -ex 'target remote localhost:2331' -ex 'load' -ex 'mon reset 0' -ex 'b main' -ex 'c'"
