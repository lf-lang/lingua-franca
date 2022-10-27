#!/usr/bin/bash

echo "Building Zephyr application"

set -e # Return on first error

# Verify command line arguments
if [ $# -lt 1 ]; then
    echo "ERROR: Please pass the board to zephyr_build.sh. e.g. ./zephyr_build.sh qemu_cortex_m3 flash"
    exit 1
fi

# Set some global variables
export BOARD=$1
export LF_SRC_DIRECTORY=$(pwd)

# Set some paths
LF_ROOT=$LF_BIN_DIRECTORY/../../..
REACTOR_C=$LF_ROOT/org.lflang/src/lib/c/reactor-c

# Copy files to avoid rebuilding all the time. This will not be necessary once we settle on a reactor-c implementation
cp $REACTOR_C/core/platform.h $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/reactor_common.c $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/reactor.c $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/reactor.h $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/threaded/reactor_threaded.c $LF_SOURCE_GEN_DIRECTORY/core/threaded
cp $REACTOR_C/core/threaded/scheduler_NP.c $LF_SOURCE_GEN_DIRECTORY/core/threaded
cp $REACTOR_C/core/platform/lf_zephyr_support.h $LF_SOURCE_GEN_DIRECTORY/core/platform
cp $REACTOR_C/core/platform/lf_zephyr_support.c $LF_SOURCE_GEN_DIRECTORY/core/platform
cp $REACTOR_C/core/platform/Platform.cmake $LF_SOURCE_GEN_DIRECTORY/core/platform

cp $REACTOR_C/core/reactor.c $LF_SOURCE_GEN_DIRECTORY/include/core
cp $REACTOR_C/core/threaded/reactor_threaded.c $LF_SOURCE_GEN_DIRECTORY/include/core/threaded
cp $REACTOR_C/core/threaded/scheduler_NP.c $LF_SOURCE_GEN_DIRECTORY/include/core/threaded
cp $REACTOR_C/core/reactor.h $LF_SOURCE_GEN_DIRECTORY/include/core
cp $REACTOR_C/core/reactor_common.c $LF_SOURCE_GEN_DIRECTORY/include/core
cp $REACTOR_C/core/platform.h $LF_SOURCE_GEN_DIRECTORY/include/core
cp $REACTOR_C/core/platform/lf_zephyr_support.h $LF_SOURCE_GEN_DIRECTORY/include/core/platform
cp $REACTOR_C/core/platform/lf_zephyr_support.c $LF_SOURCE_GEN_DIRECTORY/include/core/platform
cp $REACTOR_C/core/platform/Platform.cmake $LF_SOURCE_GEN_DIRECTORY/include/core/platform

# Insert the zephyr find_package BEFORE project defintion. This important 
#   see here: https://github.com/zephyrproject-rtos/zephyr/issues/18906?imz_s=ev33andn83vnopooshfrvkg7l6
sed -i '/^project/i \
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE}) \
' $LF_SOURCE_GEN_DIRECTORY/CMakeLists.txt

# Set the LF_QEMU_EMULATION flag if our board is qemu. It makes us NOT use HW timer but the sysclock which is less precise
if [[ "$BOARD" == "qemu"* ]]; then 
echo "
target_compile_definitions(app PUBLIC LF_QEMU_EMULATION)
" >> $LF_SOURCE_GEN_DIRECTORY/zephyr.cmake
fi

# FIXME: We have to hardcode HEAP_MEM_POOL size. Now it is 4kB
printf '# Auto-generated Config
# Enable printk
CONFIG_PRINTK=y
# Set the memory pool sufficently large.
CONFIG_HEAP_MEM_POOL_SIZE=4096
# Enable FP printing
CONFIG_CBPRINTF_FP_SUPPORT=y
# Disable preemptive scheduling
# CONFIG_NUM_PREEMPT_PRIORITIES=0
# Enable counters (used as HW clock)
CONFIG_COUNTER=y
# Have counter use the HW timer as basis
CONFIG_COUNTER_TIMER1=y
# Enable GPIO
CONFIG_GPIO=y
' > $LF_SOURCE_GEN_DIRECTORY/prj.conf

# FIXME: This config is not understood. What is Kconfig.zephyr.
printf '# Auto-generated config
source "Kconfig.zephyr"
' > $LF_SOURCE_GEN_DIRECTORY/Kconfig

cd $LF_SOURCE_GEN_DIRECTORY

# Build project
west build -b $BOARD 

if [[ "$2" == "flash" ]]; then

if [[ "$BOARD" == "nrf"* ]]; then
    echo "--- Flashing to NRF board"
    # Flash application
    bash $LF_SRC_DIRECTORY/scripts/zephyr_flash_nrf.sh .

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

elif [[ "$2" == "debug" ]]; then

if [[ "$BOARD" == "nrf"* ]]; then
    echo "--- Debugging NRF board"

    # Debug application
    bash $LF_SRC_DIRECTORY/scripts/zephyr_debug_nrf.sh .

fi

fi