#!/bin/bash

echo "Creating udev rules for IMU and RPLIDAR"

# Source colcon_cd function if needed
echo "remap the device serial port(ttyUSB1) -> rplidar, port(ttyUSB0) -> imu_usb"
echo "check it using the command : ls -l /dev | grep ttyUSB"

# Copy the rules file to /etc/udev/rules.d/
echo "Started copy the rule..."
sudo cp /home/ubuntu/robot_ws/src/integrated_robot_control/rules/sensor.rules /etc/udev/rules.d/
echo "Finished copy the rule..."

echo "Reloading and restarting udev service..."
sudo service udev reload
sleep 2
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger

echo "Finish creating udev rules"
