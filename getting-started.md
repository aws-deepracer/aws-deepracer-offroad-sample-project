# DeepRacer Offroad Sample Project

The DeepRacer Offroad sample project is a sample application built on top of the existing AWS DeepRacer application that adds additional capability to the DeepRacer device to autonomously navigate through a path of QR codes by decoding the information in them. Explore the DeepRacer Offroad sample project by cloning the [aws-deepracer-offroad-sample-project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project) and get started using the provided [DeepRacer Offroad Launcher Package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_launcher).


## Overview

The DeepRacer Offroad sample project showcases a new way to autonomously navigate a DeepRacer around
a custom path built using a series of QR codes as waypoints. In order to move along the path, the DeepRacer decodes these QR codes created by you and calculates the direction in which it has to move in the next step. You can create your own custom path by placing the waypoint codes as a sequence along the track.

We can encode different kinds of information in these waypoint codes and develop logic to get the DeepRacer respond accordingly. At the basic level, the Offroad sample project requires the QR codes to provide track edge information based on their position value(left/right) encoded in them. This application also provides logic to mimic a set of special actions(“Go Straight”, “Hairpin right”, “Hairpin left”, etc.) using detection delta information and action limit values (more details below) encoded in these waypoints. 

The decoded information in the basic waypoint QR codes is in the format below. Example:

        'DR: {"wp": 1, "p": "l"}'

The following information are encoded in the JSON object shown above:

1. **Waypoint number (wp)**: Waypoint numbers are integers that can be used to enforce the ordering in which the QR codes are to be considered while deciding the action. In this version of Offroad sample project, the waypoint numbers are used to plot the path connecting the ordered sequence of the QR code reference points in the display camera stream.

1. **Position of the QR code (p)**: The position value(left: l, right: r) indicate the track edge position of the QR code. It is helpful in plotting the QR code reference point beside the QR code so that the DeepRacer can pass it on the correct side.

**Special actions:**
This application also provides logic to respond to *“special action”* waypoint codes as an example to showcase ways to extend the application. Example:

        'DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [0.0, -0.2], "limit": 35}'

The following additional information are encoded in the JSON object shown above:

1. **Type of the waypoint (type)**: Apart from the basic waypoints, we can also have special waypoint types like *action_start* and *action_stop*. *action_start* type informs the Offroad application to continue to publish the delta value passed as an background ongoing action until an *action_stop* waypoint is encountered. This allows us to build high level actions like straight (delta: [0.0, -0.2]), hairpin_right (delta: [1.0., -0.2]), hairpin_left (delta: [-1.0., -0.2]), etc,.

1. **Limit the ongoing action (limit)**: This is a positive integer value [1, infinity] indicating the maximum number of image frames that the ongoing action triggered by *action_start* waypoint should be executed. This limit value provides a finer control over these special actions so that you can stop the background action after a certain time.

Setting these track edge and special actions in a series allows you to create a custom path for the DeepRacer to follow. The hand-off between the waypoints as the DeepRacer goes past the waypoint codes is handled in the qr_detection_node. To learn more about how to create/extend the QR Codes for your use case, use the [sample guide](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md) and invent your track!


## How does is work?

The DeepRacer Offroad application uses many nodes from the AWS DeepRacer core application as is and adds a few specific extension inspired from the Follow the Leader sample project to the shared nodes. This application is built to work alongside the AWS DeepRacer core application so that we can run both of the applications simultaneously.

* **DeepRacer Core Packages used without modification**
    * camera_pkg
    * deepracer_navigation_pkg
    * deepracer_systems_pkg
    * device_info_pkg
    * i2c_pkg
    * inference_pkg
    * model_optimizer_pkg
    * sensor_fusion_pkg
    * servo_pkg
    * status_led_pkg
    * usb_monitor_pkg
* **DeepRacer Core Packages modified to support DeepRacer Offroad**
    * webserver_pkg
    * ctrl_pkg
    * deepracer_interfaces_pkg
* **DeepRacer Off-road functionality specific packages**
    * qr_detection_pkg (modified from object_detection_pkg in Follow the Leader(FTL) sample Project)
    * deepracer_offroad_navigation_pkg (modified from ftl_navigation_pkg in Follow the Leader(FTL) sample Project)


## Hardware Setup

The Offroad sample project is built to work on *AWS DeepRacer* with a single camera attached to it.


## Main Components

There are six packages (ROS Nodes) that are of importance for the DeepRacer Offroad sample project.
 
1. [QR Detection Pkg](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/qr_detection_pkg) - Package responsible to detect QR Code from the camera sensor images and calculate the error (delta) in displacement of the QR code reference point from target position to be provided to the DeepRacer Offroad navigation.

1. [DeepRacer Offroad Navigation Pkg](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_navigation_pkg) - Package responsible for collecting the delta results from qr detection and mapping it to the servo message with throttle and steering angle values.

1. [DeepRacer Offroad Launcher Package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_launcher) - The DeepRacer Offroad Launcher Package is the package responsible for launching all the required nodes for the DeepRacer Offroad sample project, including the launch setup for nodes from the AWS DeepRacer core application.

1. [Control Package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/ctrl_pkg) - Package extended from AWS DeepRacer core application and responsible for creating main node with services exposed to be used by webserver backend API calls. This manages the mode of the car: manual, autonomous, calibration or offroad.

1. [DeepRacer Interfaces Package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_interfaces_pkg) - The DeepRacer Interfaces ROS package is a foundational package that creates the custom service and message types that are used in the Deepracer Offroad sample project.

1. [Webserver Package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/webserver_pkg) - Package extended from AWS DeepRacer core application and responsible for creating a collection of FLASK APIs that are called from the front end. These APIs call the required ROS services and return the result to the front end required for Deepracer Offroad sample project to interact with the device console.


## offroad mode:

The Deepracer Offroad sample project introduces a new mode (offroad mode) of operation in the AWS DeepRacer device apart from the existing modes of operation(autonomous mode, calibration mode and manual mode). More details about the existing modes of operation in the AWS DeepRacer device is found [here](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).  

In the offroad mode the DeepRacer devices takes the camera image input from the front facing camera connected to the car and runs QR Detection to identify a QR Code, decode it and calculate information required to plan its action. Similar to the autonomous mode, there is an perception-decision-action step involved here as well, where the decision step involves decoding information from the waypoint QR code and calculating appropriate reference deltas. This decoding is done by QR Detection python wrapper [pyzbar](https://pypi.org/project/pyzbar/) to obtain the bounding box data for the QR code identified in the image and decode the information embedded in it. Each perception-decision-action step involves a pipeline of a series of ROS messages published/subscribed at various nodes, to publish camera image and then to publish the qr detection deltas identifying QR Code’s position and corresponding action data to go past it.

![deepracer-offroad-flow](/media/deepracer-offroad-flow.png)


### Decision

The decision step is handled by the QR detection ROS package that creates the qr_detection_node responsible for collecting sensor data (camera images) from sensor_fusion_pkg and running qr detection on the images. A target point is initialized in the qr_detection_node that acts as a source reference for the DeepRacer to calculate the detection error (delta) whenever QR code is detected. As part of this step, the node publishes the normalized delta data from target reference point to the waypoint reference point as DetectionDeltaMsg data, indicating the direction in which the DeepRacer is supposed to move.  

For each input image, the node will try to detect and decode a QR code. Based on the decoded data (Position/Limit/Delta etc), it will calculate CUR_IMG_REF_POINT (Point from where it can pass the QR Code safely) and calculate the (x, y) delta [CUR_IMG_REF_POINT - TARGET_REFERENCE_POINT] as shown in the figure below. The CUR_IMG_REF_POINT depends on the position of the QR code, for example: 

1. If the position encoded in the QR code is “left”:
   
   The reference point would considered to be on the right of the QR Code, so that the DeepRacer can pass it with the identified QR code to its left.

   ![deepracer-offroad-qr-detection-bb-left](/media/deepracer-offroad-qr-detection-bb-left.png)

1. If the position encoded is “right”:
   
   The reference point would considered to be on the left of the QR Code for the DeepRacer to pass it with the identified QR code to the right.

   ![deepracer-offroad-qr-detection-bb-right](/media/deepracer-offroad-qr-detection-bb-right.png)

1. When there are multiple QR Codes detected (both left and right positioned)

   ![deepracer-offroad-qr-detection-bb-multipleqr](/media/deepracer-offroad-qr-detection-bb-multipleqr.png)

This delta value is published as the DetectionDeltaMsg data to /qr_detection_pkg/qr_detection_delta topic whic will be read by the DeepRacer Offroad navigation node.

If no QR code is detected in a 3 consecutive image, the qr_detection_node publishes a zero error (delta) signifying that the DeepRacer is already at the target position and need not move, unless it encounters a QR with Limit set. 


### Action (Navigation)

The DeepRacer Offroad Navigation ROS package creates the deepracer_offroad_navigation_node which decides the action / controller message to send out based on the normalized detection error (delta) received from qr_detection_node. The node uses a very simple action space to account for the various combinations of the (x, y) delta values that are expected.

#### Define the valid DetectionDelta values:

![deepracer-offroad-navigation-moves](/media/deepracer-offroad-navigation-moves.png)

As an initial step in coming up with the action space to be used for navigation, we have to define valid detection delta ranges that will be supported by the application. The above diagram shows the 6 valid {delta_x, delta_y} ranges that can be passed to the deepracer_offroad_navigation_node. The deepracer_offroad_navigation_node is designed to work with a negative range of delta_y values to ensure the DeepRacer device keeps moving on looking for the next waypoint code by taking forward actions when a waypoint code is detected.

These {delta_x, delta_y} values are the difference in the Target position to the CUR_IMG_REF_POINT calculated from the QR in the current image. The different detection delta values can be grouped into these high level actions that can be further used to define more granular actions:

|   Case    |    Steering    |    Throttle    |
| --------- | -------------- | -------------- |
|     1     |      Left      |     Forward    |
|     2     |      NULL      |     Forward    |
|     3     |      Right     |     Forward    |
|     4     |      Left      |     NULL       |
|     5     |      NULL      |     NULL       |
|     6     |      Right     |     NULL       |

#### Define the specific action values:

While implementing the navigation logic, it is important to build a safe and meaningful action space with right granularity in steering and throttle actions. We need to map the DetectionDelta values passed to the deepracer_offroad_navigation_node to specific steering and throttle values to be sent to servo of the DeepRacer. These specific steering and throttle values are obtained by defining bounding brackets with different thresholds of the {delta_x, delta_y} ranges.

The granular action values and their corresponding {delta_x, delta_y} threshold values used in the application has been defined by empirically collecting the {delta_x, delta_y} value of the QR Codes at different positions with respect to the camera of the DeepRacer device. These take into account the distance of the waypoint detected (CUR_IMG_REF_POINT) from current TARGET position and the direction the DeepRacer needs to turn to pass the waypoint at a safe distance. The grid diagram below shows a top down view of the placement positions for the CUR_IMG_REF_POINT and Target point (Camera).

![deepracer-offroad-qr-detection-delta-nav-mapping](/media/deepracer-offroad-qr-detection-delta-nav-mapping.png)

These {delta_x, delta_y} values with respect to the CUR_IMG_REF_POINT from the camera TARGET enables us to create a safe distance bracket for valid actions. These brackets are then mapped to the steering and the speed values required by the DeepRacer servo node. Based on the data collected, we get the following action brackets:

**Steering:**

* DEFAULT: No steering, when the CUR_IMG_REF_POINT is on the straight line of sight with respect to camera
* MIN_LEFT
* MID_LEFT
* MAX_LEFT
* MIN_RIGHT
* MID_RIGHT
* MAX_RIGHT

**Speed:**

* DEFAULT: No throttle
* FORWARD

Every combination of the normalized delta values (delta_x and delta_y) are mapped to their corresponding brackets and a steering and throttle value is published to the servo node. Here, the logic in navigation which plans an action based on the Delta combinations received from qr_detection_node can be changed to add more actions to define your own action space as per the use case you have.

Hence, using this pipeline for Perception - Decision - Action on a loop, the DeepRacer detects QR Codes, plans what action is needed to bring the CUR_IMG_REF_POINT at the target position and takes the action for each image it decodes, thus achieving the goal creating a path through a series of QR Codes placed on the ground.


## Demo

First person view of how the DeepRacer navigates through a setup path:

![deepracer-offroad-demo](/media/deepracer-offroad-demo.gif)


## Possible next steps:

The DeepRacer Offroad project is an example of how the individual nodes in Follow the Leader(FTL) sample project were modified to achieve a different goal. The object_detection_node has been modified to replace the Machine Learning inference step using an Object Detection Model with a simple python wrapper for QR Code Detection. Also, with few tweaks to the ftl_navigation_node, we implement the deepracer_offroad_navigation_node which handles the action planning based on Detection Deltas from qr_detection_node.

This way, the individual nodes used in the DeepRacer Offroad sample project or the entire sample project can be used to develop a new feature or a concept of racing using the QR Codes.

Some ideas that you may explore are listed below:

* **Find your way out -** Setup the QR codes as a maze and encode instructions in them to let DeepRacers decide the path it wants to take.
* **Record and replay -** Add memory store to save actions from initial identification of the a special waypoint type, and rerun them when encountering the same waypoint type.
* **Build your own track and race!!**


## Summary

The DeepRacer Offroad sample project leverages most of the concepts used in the AWS DeepRacer application You can learn more about the AWS DeepRacer core application [here](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).


## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Instructions to create QR Codes to setup Offroad path: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md)
