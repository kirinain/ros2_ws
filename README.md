# ros2_ws

## **1. Build the Package**
Run the following command to build the package:
```bash

colcon build --symlink-install
```
## **2. Source the Setup File**


After a successful build, source the setup file:
```bash


source install/setup.bash
```

## **3. Set Up the TurtleBot3 Simulation**

Set the TurtleBot3 model to waffle_pi and launch the house simulation:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
## **4. Run Waypoint Navigation**

Use the differential_drive_controller to navigate through waypoints:
```bash
ros2 run differential_drive_controller waypoints_nav.py --ros-args \
  -p waypoint_1_x:=2.0 \                                
  -p waypoint_1_y:=1.0 \
  -p waypoint_2_x:=4.0 \
  -p waypoint_2_y:=3.0 \
  -p kp:=0.5 \
  -p ki:=0.0 \
  -p kd:=0.1 \
  -p safe_distance:=0.01
```
## **5. Monitor Wheel RPMs**

Check the left and right wheel RPM values:
```bash
ros2 topic echo /left_wheel_rpm                                     
ros2 topic echo /right_wheel_rpm
```
## **6. Run the Drive Controller**

Start the drive controller using:
```bash
ros2 run differential_drive_controller drive_controller
```

---
