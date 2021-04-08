# AWS DeepRacer Offroad Launcher Package

## Overview

The DeepRacer Offroad sample project is a sample application built on top of the existing AWS DeepRacer application which uses QR Code detection through which the AWS DeepRacer device can identify and navigate through a path setup using QR Codes to guide the car. For detailed information on DeepRacer Offroad sample project, see DeepRacer Offroad sample project [Getting Started](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md) section.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the DeepRacer Offroad sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page. DeepRacer Offroad sample project requires the AWS DeepRacer application to be installed on the device as it leverages most of the packages from the core application.

The following are the additional software requirements to get the DeepRacer Offroad sample project to work on the AWS DeepRacer device.

1. **Calibrate the AWS DeepRacer (optional):** Follow the [instructions](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) to calibrate the mechanics of your AWS DeepRacer Vehicle. This should be done so that the vehicle performance is optimal and it behaves as expected.


## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Stop the deepracer-core.service that is currently running on the device:

        systemctl stop deepracer-core

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

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the packages in the workspace

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/ && colcon build


## Usage

### Run the node

To launch the DeepRacer Offroad sample application as root user on the AWS DeepRacer device open up another terminal on the device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash

1. Launch the nodes required for DeepRacer Offroad sample project:

        ros2 launch deepracer_offroad_launcher deepracer_offroad_launcher.py

### Enabling “offroad” mode using CLI:

Once the deepracer_offroad_launcher has been kicked-off, open up a adjacent new terminal as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash

1. Set the mode of the DeepRacer via ctrl_pkg to “offroad” using the below ros2 service call:

        ros2 service call /ctrl_pkg/vehicle_state deepracer_interfaces_pkg/srv/ActiveStateSrv "{state: 3}"

1. Enable “offroad” mode using the below ros2 service call

        ros2 service call /ctrl_pkg/enable_state deepracer_interfaces_pkg/srv/EnableStateSrv "{is_active: True}"

### Changing the MAX_SPEED scale of the DeepRacer:

The MAX_SPEED scale of the DeepRacer can be modified using ros2 service call in case the car isn’t moving as expected. This can occur due to multiple reasons not limited to vehicle battery percentage, surface on which the car is being operated etc.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the DeepRacer Offroad workspace:

        cd ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-offroad-sample-project/deepracer_offroad_ws/install/setup.bash

1. Change the MAX SPEED to xx% of the MAX Scale:

        ros2 service call /deepracer_offroad_navigation_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.xx}"

    Example: Change the MAX SPEED to 75% of the MAX Scale:

        ros2 service call /deepracer_offroad_navigation_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.75}"


## Launch Files

The deepracer_offroad_launcher.py included in this package is the main launcher file that launches all the required nodes for the DeepRacer Offroad sample project. This launcher file also includes the nodes from the AWS DeepRacer core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            ld = LaunchDescription()
            qr_detection_node = Node(
                package='qr_detection_pkg',
                namespace='qr_detection_pkg',
                executable='qr_detection_node',
                name='qr_detection_node'
            )
            deepracer_offroad_navigation_node = Node(
                package='deepracer_offroad_navigation_pkg',
                namespace='deepracer_offroad_navigation_pkg',
                executable='deepracer_offroad_navigation_node',
                name='deepracer_offroad_navigation_node'
            )
            camera_node = Node(
                package='camera_pkg',
                namespace='camera_pkg',
                executable='camera_node',
                name='camera_node'
            )
            ctrl_node = Node(
                package='ctrl_pkg',
                namespace='ctrl_pkg',
                executable='ctrl_node',
                name='ctrl_node'
            )
            deepracer_navigation_node = Node(
                package='deepracer_navigation_pkg',
                namespace='deepracer_navigation_pkg',
                executable='deepracer_navigation_node',
                name='deepracer_navigation_node'
            )
            software_update_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='software_update_node',
                name='software_update_node'
            )
            model_loader_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='model_loader_node',
                name='model_loader_node'
            )
            otg_control_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='otg_control_node',
                name='otg_control_node'
            )
            network_monitor_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='network_monitor_node',
                name='network_monitor_node'
            )
            device_info_node = Node(
                package='device_info_pkg',
                namespace='device_info_pkg',
                executable='device_info_node',
                name='device_info_node'
            )
            battery_node = Node(
                package='i2c_pkg',
                namespace='i2c_pkg',
                executable='battery_node',
                name='battery_node'
            )
            inference_node = Node(
                package='inference_pkg',
                namespace='inference_pkg',
                executable='inference_node',
                name='inference_node'
            )
            model_optimizer_node = Node(
                package='model_optimizer_pkg',
                namespace='model_optimizer_pkg',
                executable='model_optimizer_node',
                name='model_optimizer_node'
            )
            rplidar_node = Node(
                package='rplidar_ros',
                namespace='rplidar_ros',
                executable='rplidarNode',
                name='rplidarNode',
                parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': True,
                    }]
            )
            sensor_fusion_node = Node(
                package='sensor_fusion_pkg',
                namespace='sensor_fusion_pkg',
                executable='sensor_fusion_node',
                name='sensor_fusion_node'
            )
            servo_node = Node(
                package='servo_pkg',
                namespace='servo_pkg',
                executable='servo_node',
                name='servo_node'
            )
            status_led_node = Node(
                package='status_led_pkg',
                namespace='status_led_pkg',
                executable='status_led_node',
                name='status_led_node'
            )
            usb_monitor_node = Node(
                package='usb_monitor_pkg',
                namespace='usb_monitor_pkg',
                executable='usb_monitor_node',
                name='usb_monitor_node'
            )
            webserver_publisher_node = Node(
                package='webserver_pkg',
                namespace='webserver_pkg',
                executable='webserver_publisher_node',
                name='webserver_publisher_node'
            )
            web_video_server_node = Node(
                package='web_video_server',
                namespace='web_video_server',
                executable='web_video_server',
                name='web_video_server'
            )
            ld.add_action(qr_detection_node)
            ld.add_action(deepracer_offroad_navigation_node)
            ld.add_action(camera_node)
            ld.add_action(ctrl_node)
            ld.add_action(deepracer_navigation_node)
            ld.add_action(software_update_node)
            ld.add_action(model_loader_node)
            ld.add_action(otg_control_node)
            ld.add_action(network_monitor_node)
            ld.add_action(device_info_node)
            ld.add_action(battery_node)
            ld.add_action(inference_node)
            ld.add_action(model_optimizer_node)
            ld.add_action(rplidar_node)
            ld.add_action(sensor_fusion_node)
            ld.add_action(servo_node)
            ld.add_action(status_led_node)
            ld.add_action(usb_monitor_node)
            ld.add_action(webserver_publisher_node)
            ld.add_action(web_video_server_node)
            return ld


## Demo

First person view of how the DeepRacer navigates through a setup path:

![deepracer-offroad-demo](/media/deepracer-offroad-demo.gif)


## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* DeepRacer Offroad sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)
* Instructions to create QR Codes to setup Offroad path: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md)
* Instructions to calibrate your AWS DeepRacer: [https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html)
