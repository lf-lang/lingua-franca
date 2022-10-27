# Erase and flash application onto NRF52
# Takes the path to the build folder as an argument
SRC_GEN_PATH=$1

NRFJPROG=nrfjprog
echo "--- Executing zephyr_flash_nrf.sh"
eval $NRFJPROG -e
eval $NRFJPROG --program $SRC_GEN_PATH/build/zephyr/zephyr.hex --sectorerase --verify
eval $NRFJPROG --reset
