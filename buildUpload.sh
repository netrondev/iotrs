#! /bin/bash

# ./buildUpload.sh usb_echo

echo ""
echo ""
echo "============================================================"
echo ""
echo ""

# rm -rf target/thumbv6m-none-eabi/debug/examples

echo "Building"
cargo build || exit 1

echo "Converting to binary"
arm-none-eabi-objcopy -O binary target/thumbv6m-none-eabi/debug/iotrs target/thumbv6m-none-eabi/debug/iotrs.bin

# echo "Setting baud rate to 1200"
# stty -F /dev/ttyACM0 ospeed 1200

echo "Uploading"
~/.arduino15/packages/arduino/tools/bossac/1.7.0-arduino3/bossac -i -d --port=ttyACM0 -U true -i -e -w -v "target/thumbv6m-none-eabi/debug/iotrs.bin" -R

echo "If you get an error about no serial port, double check the port. Press the reset button twice and it should appear"