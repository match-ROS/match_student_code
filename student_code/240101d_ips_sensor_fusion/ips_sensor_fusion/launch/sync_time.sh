#! /bin/sh
#script to synchronize time between scout and mobilemaster
#only works if sudo password request is turned off
echo "synchronizing time"
ssh agilex sudo date -s @$(date -u +"%s") #actual synchronization
scout_time=$(ssh agilex date) #get scout time
mobilemaster_time=$(date)   #get mobilemaster time

echo "~~~~~~~~~~~~~~~~~~~"
echo "scout time:"
echo $scout_time
echo "mobilemaster time:"
echo $mobilemaster_time
echo "~~~~~~~~~~~~~~~~~~~"
echo "done"
exit 0