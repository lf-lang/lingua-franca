echo "Building Zephyr application"

export BOARD=nrf52dk_nrf52832

set -e # Return on first error
LF_ROOT=$LF_BIN_DIRECTORY/../../..
REACTOR_C=$LF_ROOT/org.lflang/src/lib/c/reactor-c
export LF_SRC_DIRECTORY=$(pwd)

# Copy files to avoid rebuilding all the time
cp $REACTOR_C/core/platform.h $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/reactor_common.c $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/reactor.c $LF_SOURCE_GEN_DIRECTORY/core
cp $REACTOR_C/core/platform/lf_zephyr_support.h $LF_SOURCE_GEN_DIRECTORY/core/platform
cp $REACTOR_C/core/platform/lf_zephyr_support.c $LF_SOURCE_GEN_DIRECTORY/core/platform
cp $REACTOR_C/core/platform/Platform.cmake $LF_SOURCE_GEN_DIRECTORY/core/platform

cp $REACTOR_C/core/reactor.c $LF_SOURCE_GEN_DIRECTORY/include/core
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


# FIXME: We have to hardcode HEAP_MEM_POOL size. Now it is 1024Bytes
printf '# Auto-generated Config
CONFIG_PRINTK=y
CONFIG_HEAP_MEM_POOL_SIZE=4096
CONFIG_CBPRINTF_FP_SUPPORT=y
' > $LF_SOURCE_GEN_DIRECTORY/prj.conf

# FIXME: This config is not understood. What is Kconfig.zephyr.
printf '# Auto-generated config
source "Kconfig.zephyr"
' > $LF_SOURCE_GEN_DIRECTORY/Kconfig

cd $LF_SOURCE_GEN_DIRECTORY
west build -b $BOARD 

# Flash application
cd build/zephyr
$LF_SRC_DIRECTORY/scripts/zephyr_flash_nrf.sh

# Debug application
# FIXME: Fix the issues here. Why isnt gdb working when invoked from this script?
# $LF_SRC_DIRECTORY/../scripts/zephyr_debug_nrf.sh