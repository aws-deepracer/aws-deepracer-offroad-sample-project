# AWS DeepRacer QR detection package

## Overview

The QR detection ROS package creates the `qr_detection_node`, which is responsible for collecting sensor data (camera images) from `sensor_fusion_pkg` to decode detected QR codes and calculate the error (delta) in displacement of the QR code reference point from the target position to be provided to the navigation. This delta value is published using a ROS publisher as `DetectionDeltaMsg` data. For more information about the DeepRacer Offroad sample project, see [DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer QR detection package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the AWS DeepRacer Offroad sample project. For more information about the pre-installed set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `qr_detection_pkg` specifically depends on the following ROS 2 packages as build and execute dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the AWS DeepRacer Offroad sample project.


## Downloading and building

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

1. Clone the entire AWS DeepRacer Offroad sample project on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project.git
        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Clone the `async_web_server_cpp`, `web_video_server`, `rplidar_ros`, and `pyzbar` dependency packages on the AWS DeepRacer device:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && ./install_dependencies.sh

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `qr_detection_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && colcon build --packages-select qr_detection_pkg deepracer_interfaces_pkg


## Usage

Although the `qr_detection_node` is built to work with the AWS DeepRacer Offroad sample project, it can be run independently for development, testing, and debugging purposes.

### Run the node

To launch the built `qr_detection_node` as the root user on the AWS DeepRacer device, open another terminal on the device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Navigate to the AWS DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS 2 Foxy setup bash and OpenVINO bash script:

        source /opt/ros/foxy/setup.bash 
        source /opt/intel/openvino_2021/bin/setupvars.sh 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash 

1. Launch the `qr_detection_node` using the launch script:

        ros2 launch qr_detection_pkg qr_detection_pkg_launch.py

## Launch files

A launch file called `qr_detection_pkg_launch.py` is included in this package that gives an example of how to launch the `qr_detection_node`.

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


## Node details

### `qr_detection_node`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `/sensor_fusion_pkg/sensor_msg` | `EvoSensorMsg` | This message holds a list of `sensor_msgs` and image objects that are independently collected from different camera sensors. |


#### Published Topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
| `qr_detection_delta` | `DetectionDeltaMsg` | Message with the QR detection normalized error (delta) of the detected QR reference from the target (reference) position with respect to `x` and `y` axes. |
| `qr_detection_display` | Image | Message to display the input stream of images after inference, published to the local `web_video_server`. |

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Getting started with the AWS DeepRacer Offroad](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)
* [Instructions for creating QR codes to set up the Offroad path](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md)
