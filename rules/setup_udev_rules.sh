#!/bin/bash

echo "Creating udev rules for IMU and RPLIDAR"

# Source colcon_cd function if needed
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
   source /usr/share/colcon_cd/function/colcon_cd.sh
     colcon_cd integrated_robot_control
fi

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
