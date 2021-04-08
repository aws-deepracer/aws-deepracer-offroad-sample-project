# AWS DeepRacer DeepRacer Offroad Navigation Package

## Overview

The DeepRacer Offroad Navigation ROS package creates the deepracer_offroad_navigation_node which decides the action / controller message to send out using the normalized detection error (delta) received from qr_detection_node. For more information about the DeepRacer Offroad sample project, see [DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the DeepRacer Offroad sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepracer_offroad_navigation_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support DeepRacer Offroad sample project.

### Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire DeepRacer Offroad sample project on the DeepRacer device.

        git clone https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepracer_offroad_navigation_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && colcon build --packages-select deepracer_offroad_navigation_pkg deepracer_interfaces_pkg


## Usage

Although the **deepracer_offroad_navigation_node** is built to work with the DeepRacer Offroad sample project, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built deepracer_offroad_navigation_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash 

1. Launch the deepracer_offroad_navigation_node using the launch script:

        ros2 launch deepracer_offroad_navigation_pkg deepracer_offroad_navigation_pkg_launch.py

## Launch Files

A launch file called deepracer_offroad_navigation_pkg_launch.py is included in this package that gives an example of how to launch the deepracer_offroad_navigation_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='deepracer_offroad_navigation_pkg',
                    namespace='deepracer_offroad_navigation_pkg',
                    executable='deepracer_offroad_navigation_node',
                    name='deepracer_offroad_navigation_node'
                )
            ])


## Node Details

### deepracer_offroad_navigation_node

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
|/qr_detection_pkg/qr_detection_delta|DetectionDeltaMsg|Message with QR Detection normalized error (delta) of the detected QR from the target position with respect to x and y axes.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|offroad_drive|ServoCtrlMsg|This message is used to send motor throttle and servo steering angle ratios with respect to the device calibration. It can also be used to send raw PWM values for angle and throttle.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|set_max_speed|SetMaxSpeedSrv|Sets Max Speed Percentage Scale for DeepRacer Offroad Application.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* DeepRacer Offroad sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)
