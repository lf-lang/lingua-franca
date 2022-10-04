# Erase and flash application onto NRF52
# Takes the path to the build folder as an argument
BUILD_PATH=$1

NRFJPROG=nrfjprog
echo "--- Executing zephyr_flash_nrf.sh"
eval $NRFJPROG -e
eval $NRFJPROG --program $BUILD_PATH/zephyr/zephyr.hex --sectorerase --verify
eval $NRFJPROG --reset
