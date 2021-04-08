# AWS DeepRacer QR Detection Package

## Overview

The QR Detection ROS package creates the qr_detection_node which is responsible for collecting sensor data (camera images) from sensor_fusion_pkg to decode detected QR Codes and calculate the error (delta) in displacement of the QR code reference point from target position to be provided to the navigation. This delta value is published using a ROS publisher as DetectionDeltaMsg data. For more information about the DeepRacer Offroad sample project, see [DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the DeepRacer Offroad sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The qr_detection_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support DeepRacer Offroad sample project.


## Downloading and Building

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

1. Clone the async_web_server_cpp, web_video_server, rplidar_ros and pyzbar dependency packages on the DeepRacer device:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && ./install_dependencies.sh

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the qr_detection_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && colcon build --packages-select qr_detection_pkg deepracer_interfaces_pkg


## Usage

Although the **qr_detection_node** is built to work with the DeepRacer Offroad sample project, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built qr_detection_node as root user on the AWS DeepRacer device, open up another terminal on the device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS2 Foxy setup bash and OpenVINO bash script:

        source /opt/ros/foxy/setup.bash 
        source /opt/intel/openvino_2021/bin/setupvars.sh 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash 

1. Launch the qr_detection_node using the launch script:

        ros2 launch qr_detection_pkg qr_detection_pkg_launch.py

## Launch Files

A launch file called qr_detection_pkg_launch.py is included in this package that gives an example of how to launch the qr_detection_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='qr_detection_pkg',
                    namespace='qr_detection_pkg',
                    executable='qr_detection_node',
                    name='qr_detection_node'
                )
            ])


## Node Details

### qr_detection_node

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| /sensor_fusion_pkg/sensor_msg | EvoSensorMsg | This message holds a list of sensor_msgs/Image objects that are independently collected from different camera sensors. |


#### Published Topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| qr_detection_delta | DetectionDeltaMsg | Message with QR Detection normalized error (delta) of the detected QR reference from the target (reference) position with respect to x and y axes. |
| qr_detection_display | Image | Message to display the input stream of images after inference, published to the local web_video_server. |

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* DeepRacer Offroad sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)
* Instructions to create QR Codes to setup Offroad path: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md)
