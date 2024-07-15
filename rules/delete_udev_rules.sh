#!/bin/bash

echo "Deleting udev rules for IMU and RPLIDAR"
sudo rm /etc/udev/rules.d/sensor.rules
echo " "
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "Finish deleting udev rules"
