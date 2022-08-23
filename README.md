# Tello Controller
This repo contains ROS2 package that uses the ROS Joy package  
to control DJI Tello drones with Logitech Extreme 3D Pro joystick.  
# How To Build
In the directory of your package, use colcon to build the package:
```bash
colcon build
```
# How To Use
1. Open a terminal, run the joy node:
```bash
ros2 run joy joy_node
```
2. In a new terminal, move to the package's built directory
```bash
cd <directory_of_package>
```
3. Then source the setup script:
```bash
source install/setup.bash
```
4. Run the package
```bash
ros2 run tello_controller controller
