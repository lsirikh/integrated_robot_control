### Integrated Robot Control Ros2 Package

* Version : v0.0.3
* Date : 2024-10-09

#### Goal
* robot_localization was applied version

1. Control the keyboard with teleop_twist_keyboard
2. Use odometry data from DC motors with encoders.  
3. IMU can be correct the accumulatable errors in the odometry of DC motor, which uses Extended Kalman Filter.  
4. CSV data can be created.  
5. ekf.yaml will be used to configurate each setting values for robot_localization pkg.  
6. CMP10A imu Ros2 Package was inserted for this project.  
7. imu.launch.py was updated to start cmp10a_imu package.   
8. ekf_robot_control pkg was not applicale fot that reason why the uncertainty was too high to be used to this project.  
7. robot_localization was applied thid project but not that impressive.  

