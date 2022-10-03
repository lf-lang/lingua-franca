# Erase and flash application onto NRF52
# assumes that we are in build/zephyr directory

NRFJPROG=nrfjprog.exe
echo "--- Executing zephyr_flash_nrf.sh"
eval $NRFJPROG -e
eval $NRFJPROG --program zephyr.hex --sectorerase --verify
eval $NRFJPROG --reset
