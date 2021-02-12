# PX4 Gazebo

## Installation

Tested on Ubuntu 18.04. under ROS Melodic.

Install PX4 Autopilot and Gazebo SITL.
```
cd <path_to_your_catkin_ws>/src
mkdir firmware
cd firmware
git clone https://github.com/antonellabarisic/PX4-Autopilot.git --recursive
cd PX4-Autopilot/
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tool
make px4_sitl gazebo DONT_RUN=1
```
Add packages to ~/.bashrc.
```
echo 'source '$(pwd)'/Tools/setup_gazebo.bash '$(pwd)' '$(pwd)'/build/px4_sitl_default' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:'$(pwd)'' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:'$(pwd)'/Tools/sitl_gazebo' >> ~/.bashrc
```
Install PX4 Gazebo.
```
cd <path_to_your_catkin_ws>/src
git clone https://github.com/antonellabarisic/px4_gazebo.git
cd ..
catkin build
source ~/.bashrc
```

## Simulation startup

### Kopterworx
```
# 1st terminal PX4 SITL
roslaunch px4_gazebo px4_sitl.launch

# 2nd terminal Gazebo 
roslaunch px4_gazebo kopterworx.launch

# 3rd terminal Mavros
roslaunch px4_gazebo mavros.launch

# 4th terminal PID carrot
roslaunch uav_ros_control pid_carrot.launch
```
Once the info ```[ecl/EKF] starting GPS fusion``` appears in the 1st terminal, the vehicle is ready to arm. To take off, type the following commands:
```
# 1st terminal PX4 SITL
commander arm
commander takeoff
```
Publish joy message in a new terminal for PID carrot.
```
rostopic pub /yellow/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
axes: [0, 0, 0, 0, 0, 0]
buttons: [0, 0, 0, 0, 0, 1]"
```
Switch to OFFBOARD mode:
```
# 1st terminal PX4 SITL
commander mode offboard
```
Proceed with standard simulation experiments...

### Iris

Everything is the same as for Kopterworx, except that PX4 and Gazebo are launched from a single file:
```
# 1st terminal PX4 SITL + Gazebo
roslaunch px4_gazebo iris.launch
```
## Adding a new model

1. **Model files**

Create a folder called ```<vehicle_name>``` under models folder and put your .sdf or .xacro files in it. Add a model.config file for your vehicle and add a folder of your model in ```models/CMakeLists.txt``` in the appropriate section. 

2. **Airframe and PX4 parameters**

Airframe configurations are stored in ```/ROMFS/px4fmu_common/init.d-posix/airframes/```. For each new model create a new file named      ```<autostart_ID>_<vehicle_name>```. Autostart ID should be unique and not start with zeros. The configuration file contains the following:
  - ```MAV_TYPE``` - MAVLink airframe type
  - ```MIXER``` - mixer file (e.g. quad_x)
  - ```NAV_RCL_ACT 0 ``` - disable RC loss failsafe mode for simulation purposes
  - other PX4 parameters (optional)

To ensure that you have a clean parameter setup after multiple parameter changes, delete folder ``` ~/.ros/eeprom```.
