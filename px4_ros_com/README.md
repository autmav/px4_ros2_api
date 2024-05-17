
# PX4 ros2 API

Developed by autmav

## Prerequisites

Tested with:

- ubuntu 20.0, ros2 (foxy), Gazebo-classic, PX4 (v1.14)

1. Installation of ros2

```
... Installation of ros2
```


2. Instalation Gazebo-classic

```
... Installation of gazebo classic
```


3. Installation of PX4-Autopilot

```
cd ~

... Installation of PX4
```

## Install

```
... creating ros2 workspace

... cloning px4_msgs in src/

git clone https://github.com/autmav/px4_ros2_api.git px4_ros_com

... cd to workspace

colcon build
```

## Test

```
# Navigate to the ws root
source install/local_setup.bash

# for taking off the drone enter the following command:
ros2 launch px4_ros_com only_takeoff.yaml

# for attitude command enter the following command:
ros2 launch px4_ros_com attitude_command.yaml

# for velocity command enter the following command:
ros2 launch px4_ros_com velocity_command.yaml

# for position command enter the following command:
ros2 launch px4_ros_com position_command.yaml
```

