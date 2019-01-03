#!/bin/bash
#
# $Id$
# Purpose: Start ntripclient

# change these 3 according to your needs
Stream='RTCM3_MAX'
User='lukea1'
Password='lukea1'
#NMEA='$GPGGA,104031,4042.3310,N,08690.3684,W,2,08,3.1,260.4,M,-32.6,M,,
USB_PATHS=$(ls -l /dev/serial/by-path)
NMEA_ARG='/dev/ttyUSB1'
RTCM_ARG='/dev/ttyUSB0'
if [[ "$USB_PATHS" =~ .*USB0.*USB1 ]]; then
	NMEA_ARG='/dev/ttyUSB1'
	RTCM_ARG='/dev/ttyUSB0'
else
	NMEA_ARG='/dev/ttyUSB0'
	RTCM_ARG='/dev/ttyUSB1'
fi

VID_CONNECTED="$(tvservice -s | grep HDMI)"
echo ${VID_CONNECTED}
if [[ $VID_CONNECTED != "" ]]; then #only run on startup if the HDMI port is not attached
	if [[ $# -ne 0 ]]; then #argument supplied in crontab to indicate startup
		exit 0
	fi
fi

DateStart=`date -u '+%s'`
SleepMin=10     # Wait min sec for next reconnect try
SleepMax=10000  # Wait max sec for next reconnect try
(while true; do
  /home/pi/ntrip_self/ntripclient -s 108.59.49.226 -r 10000 -u $User -p $Password -T 1 -Y n -A 8 -M 3 -m $Stream -e $NMEA_ARG -t $RTCM_ARG
  if test $? -eq 0; then DateStart=`date -u '+%s'`; fi
  DateCurrent=`date -u '+%s'`
  SleepTime=`echo $DateStart $DateCurrent | awk '{printf("%d",($2-$1)*0.02)}'`
  if test $SleepTime -lt $SleepMin; then SleepTime=$SleepMin; fi
  if test $SleepTime -gt $SleepMax; then SleepTime=$SleepMax; fi
  # Sleep 2 percent of outage time before next reconnect try
  sleep $SleepTime
done) 

