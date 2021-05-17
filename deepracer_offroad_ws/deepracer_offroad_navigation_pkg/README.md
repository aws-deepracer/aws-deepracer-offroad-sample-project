# AWS DeepRacer Offroad navigation package

## Overview

The AWS DeepRacer Offroad navigation ROS package creates the `deepracer_offroad_navigation_node` which decides the action or controller message to send out using the normalized detection error (delta) received from `qr_detection_node`. For more information about the AWS DeepRacer Offroad sample project, see [DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer Offroad navigation package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the DeepRacer Offroad sample project. For more information about the pre-installed set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `deepracer_offroad_navigation_pkg` specifically depends on the following ROS 2 packages as build and execute dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the AWS DeepRacer Offroad sample project.

### Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire AWS DeepRacer Offroad sample project on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `deepracer_offroad_navigation_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && colcon build --packages-select deepracer_offroad_navigation_pkg deepracer_interfaces_pkg


## Usage

Although the `deepracer_offroad_navigation_node` is built to work with the AWS DeepRacer Offroad sample project, it can be run independently for development, testing, and debugging purposes.

### Run the node

To launch the built `deepracer_offroad_navigation_node` as the root user on the AWS DeepRacer device open up another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the AWS DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash 

1. Launch the `deepracer_offroad_navigation_node` using the launch script:

        ros2 launch deepracer_offroad_navigation_pkg deepracer_offroad_navigation_pkg_launch.py

## Launch files

A launch file called `deepracer_offroad_navigation_pkg_launch.py` is included in this package. It gives an example of how to launch the `deepracer_offroad_navigation_node`.

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


## Node details

### `deepracer_offroad_navigation_node`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
|`/qr_detection_pkg/qr_detection_delta`|`DetectionDeltaMsg`|Message with the QR detection normalized error (delta) of the detected QR from the target position with respect to `x` and `y` axes.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|`offroad_drive`|`ServoCtrlMsg`|This message is used to send motor throttle and servo steering angle ratios with respect to the device calibration. It can also be used to send raw PWM values for angle and throttle.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`set_max_speed`|`SetMaxSpeedSrv`|Sets max speed percentage scale for the AWS DeepRacer Offroad Application.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the AWS DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)
