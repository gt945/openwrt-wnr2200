#!/bin/sh
BLINK=1
RUN=1
while [ $RUN -eq 1 ];
do

if hostapd_cli wps_get_status | grep "PBC Status" | grep -q Active;
then
        if [ $BLINK -eq 1 ];
        then
                echo timer > /sys/devices/platform/leds-gpio/leds/netgear:green:wps/trigger
                BLINK=0
        fi
        sleep 1
else
        echo none > /sys/devices/platform/leds-gpio/leds/netgear:green:wps/trigger
        RUN=0
fi
done