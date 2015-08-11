#!/bin/sh
while ! ls /dev | grep -c ACM;
do
killall -9 program.elf
killall -9 lidar

echo Unloading dwc_otg.ko
rmmod dwc_otg > /dev/null
echo Unloading cdc-acm.ko
rmmod cdc-acm > /dev/null

echo Enabling USB Port
gpio 127 -d ho 1
gpio 127 -d i

export LD_PRELOAD=/custom/libioctl_arm.so

echo Loading dwc_otg.ko
insmod /custom/dwc_otg.ko

echo Loading cdc-acm.ko
insmod /custom/cdc-acm.ko
sleep 1

/bin/program.elf >/dev/null&
done
