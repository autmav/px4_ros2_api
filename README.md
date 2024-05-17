
# PX4 ros2 API

Developed by autmav

## Prerequisites

Tested with:

- ubuntu 20.0, ros2 (foxy), Gazebo-classic (v: ...), PX4 (v1.14). Remember to have `PX4-Autopilot` root in your home dir.


## Setup

```bash
mkdir ~/ws
cd ~/ws
git clone https://github.com/autmav/px4_ros2_api.git src
cd ..
source /opt/ros/foxy/setup.bash
colcon build
```

## Run
Each of the following fields must be done in separate terminals.
PX4: 
```bash
cd ~/PX4_Autopilot
make px4_sitl gazebo-classic
```

XRCE: 
```bash
MicroXRCEAgent udp4 -p 8888
```

Main code: 
```bash
cd ~/ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

# for just take off
ros2 launch px4_ros_com only_takeoff.yaml

# for position mode
ros2 run px4_ros_com position_command

# for velocity mode
ros2 run px4_ros_com velocity_command

# for attitude mode
ros2 run px4_ros_com attitude_command
```

Plot: 
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run py_pubsub minimal_subscriber
```





<!-- ```bash
source install/local_setup.bash


# for taking off the drone enter the following command:
ros2 launch px4_ros_com only_takeoff.yaml

# for attitude command enter the following command:
ros2 launch px4_ros_com attitude_command.yaml

# for velocity command enter the following command:
ros2 launch px4_ros_com velocity_command.yaml

# for position command enter the following command:
ros2 launch px4_ros_com position_command.yaml


``` -->

