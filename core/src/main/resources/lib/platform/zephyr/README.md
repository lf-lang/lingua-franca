# Zephyr platform files
These are files needed to compile LF programs for the Zephyr target.
All of the files in the `boards` directory are for enabling a Timer peripheral for different devices so that we can use a Counter device for time-keeping. These device tree config overlays are copied from the `alarm` example found in the zephyr source tree under `samples/drivers/counter/alarm`.
