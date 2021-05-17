# AWS DeepRacer Offroad sample project

The AWS DeepRacer Offroad sample project is a sample application built on top of the existing AWS DeepRacer application that adds additional capability to the AWS DeepRacer device to autonomously navigate through a path of QR codes by decoding the information in them. Explore the AWS DeepRacer Offroad sample project by cloning the [aws-deepracer-offroad-sample-project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project) and get started using the provided [DeepRacer Offroad launcher package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_launcher).


## Overview

The AWS DeepRacer Offroad sample project showcases a new way to autonomously navigate an AWS DeepRacer device around
a custom path built using a series of QR codes as waypoints. In order to move along the path, the AWS DeepRacer decodes these QR codes and calculates the direction to move in the next step. You can create your own custom path by placing the waypoint codes as a sequence along the track.

You can encode different kinds of information in these waypoint codes and develop logic to get the AWS DeepRacer to respond accordingly. At the basic level, the Offroad sample project requires the QR codes to provide track edge information based on the position value (left or right) encoded in them. This application also provides logic to mimic a set of *special actions* (“Go straight”, “Hairpin right”, “Hairpin left”, etc.) using detection delta information and action limit values (more details below) encoded in these waypoints. 

The decoded information in the basic waypoint QR codes is in the following format:

        'DR: {"wp": 1, "p": "l"}'

The following information is encoded in the preceding JSON object:

1. **Waypoint number (wp)**: Waypoint numbers are integers that can be used to enforce the ordering in which the QR codes are to be considered while deciding the action. In this version of the Offroad sample project, the waypoint numbers are used to plot the path connecting the ordered sequence of the QR code reference points in the display camera stream.

1. **Position of the QR code (p)**: The position value (left: l, right: r) indicates the track edge position of the QR code. It is helpful in plotting the QR code reference point beside the QR code so that the AWS DeepRacer can pass it on the correct side.

**Special actions:**
This application also provides logic to respond to special action waypoint codes as an example to showcase ways to extend the application, as shown in the following example:

        'DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [0.0, -0.2], "limit": 35}'

The following additional information is encoded in the preceding JSON object:

1. **Type of waypoint (type)**: Apart from the basic waypoints, we can also have special waypoint types like `action_start` and `action_stop`. The `action_start` type informs the Offroad application to continue to publish the delta value passed as an background ongoing action until it encounters an `action_stop` waypoint. This allows us to build high-level actions like `straight` (delta: [0.0, -0.2]), `hairpin_right` (delta: [1.0., -0.2]), and `hairpin_left` (delta: [-1.0., -0.2]).

1. **Limit the ongoing action (limit)**: This is a positive integer value [1, infinity] which enables you to set a threshold for the number of image frames captured by the AWS DeepRacer's camera before it changes from the ongoing action, initiated by the `action_start` waypoint, to its previous action. Once `action_start` begins, as DeepRacer runs the ongoing action, it counts image frames until it reaches the threshold you’ve set. Then, the car reverts to its previous action, which can be whatever you set it to be. For example, if the previous action is `stop` and the ongoing action with a limit is `go_straight`, the AWS DeepRacer goes straight until the limit is reached and then stops. 

Setting these track edge and special actions in a series allows you to create a custom path for the AWS DeepRacer to follow. The hand-off between the waypoints as the AWS DeepRacer goes past the waypoint codes is handled in the `qr_detection_node`. To learn more about how to create or extend the QR codes for your use case, use the [sample guide](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md) and invent your track!


## How does it work?

The AWS DeepRacer Offroad application uses many nodes from the AWS DeepRacer core application as is and adds extensions inspired by the Follow the Leader sample project to the shared nodes. This application is built to work alongside the AWS DeepRacer core application so that you can run both of the applications simultaneously.

* **AWS DeepRacer core packages used without modification**
    * `camera_pkg`
    * `deepracer_navigation_pkg`
    * `deepracer_systems_pkg`
    * `device_info_pkg`
    * `i2c_pkg`
    * `inference_pkg`
    * `model_optimizer_pkg`
    * `sensor_fusion_pkg`
    * `servo_pkg`
    * `status_led_pkg`
    * `usb_monitor_pkg`
* **AWS DeepRacer core packages modified to support AWS DeepRacer Offroad**
    * `webserver_pkg`
    * `ctrl_pkg`
    * `deepracer_interfaces_pkg`
* **AWS DeepRacer Offroad functionality-specific packages**
    * `qr_detection_pkg` (modified from `object_detection_pkg` in the Follow the Leader (FTL) sample project)
    * `deepracer_offroad_navigation_pkg` (modified from `ftl_navigation_pkg` in the Follow the Leader (FTL) sample project)


## Hardware setup

The Offroad sample project is built to work on an AWS DeepRacer device with a single camera attached to it.


## Main components

There are six packages (ROS nodes) that are of importance for the AWS DeepRacer Offroad sample project.
 
1. [QR detection package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/qr_detection_pkg): This package is responsible for detecting QR codes from the camera sensor images and calculating the error (delta) in displacement of the QR code reference point from the target position to be provided to the AWS DeepRacer Offroad navigation.

1. [AWS DeepRacer Offroad navigation package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_navigation_pkg): This package is responsible for collecting the delta results from QR detection and mapping them to the servo message with throttle and steering angle values.

1. [AWS DeepRacer Offroad launcher package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_offroad_launcher): This package is responsible for launching all the required nodes for the AWS DeepRacer Offroad sample project, including the launch setup for nodes from the AWS DeepRacer core application.

1. [Control package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/ctrl_pkg): This package is extended from the AWS DeepRacer core application and responsible for creating the main node with services exposed to be used by webserver backend API calls. It manages the mode of the car: `manual`, `autonomous`, `calibration`, or `offroad`.

1. [AWS DeepRacer interfaces package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/deepracer_interfaces_pkg): This package is a foundational package that creates the custom service and message types that are used in the AWS Deepracer Offroad sample project.

1. [Webserver package](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/tree/main/deepracer_offroad_ws/webserver_pkg): This package is extended from AWS DeepRacer core application and responsible for creating a collection of FLASK APIs that are called from the front end. These APIs call the required ROS services and return the result to the front end required for the AWS Deepracer Offroad sample project to interact with the device console.


## Offroad mode:

The AWS Deepracer Offroad sample project introduces a new mode of operation (`offroad` mode) in the AWS DeepRacer device in addition to the existing modes of operation (`autonomous mode`, `calibration mode`, and `manual` mode). For more information about the existing modes of operation in the AWS DeepRacer device, see  [AWS DeepRacer application: Modes of operation](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/modes-of-operation.md).  

In the `offroad` mode, the AWS DeepRacer device takes the camera image input from the front-facing camera connected to the car and runs QR detection to identify a QR code, decode it, and calculate the information required to plan its action. As in the `autonomous` mode, there is an perception-decision-action step involved here as well: the decision step involves decoding information from the waypoint QR code and calculating appropriate reference deltas. This decoding is done by the QR detection Python wrapper [pyzbar](https://pypi.org/project/pyzbar/) to obtain the bounding box data for the QR code identified in the image and decode the information embedded in it. Each perception-decision-action step involves a pipeline of a series of ROS messages published or subscribed at various nodes. These messages publish the camera image, the QR detection deltas identifying the QR code’s position, and the corresponding action data to go past it.

![deepracer-offroad-flow](/media/deepracer-offroad-flow.png)


### Decision

The decision step is handled by the QR detection ROS package that creates the `qr_detection_node` responsible for collecting sensor data (camera images) from `sensor_fusion_pkg` and running QR detection on the images. The `qr_detection_node` initializes a target point that acts as a source reference for the AWS DeepRacer to calculate the detection error (delta) whenever it detects a QR code. As part of this step, the node publishes the normalized delta data from the target reference point to the waypoint reference point as `DetectionDeltaMsg` data, indicating the direction in which the AWS DeepRacer is supposed to move.  

For each input image, the node tries to detect and decode a QR code. Based on the decoded data (such as position, limit, or delta), it calculates `CUR_IMG_REF_POINT` (the point from where it can pass the QR code safely) and calculates the (`x`, `y`) delta [CUR_IMG_REF_POINT - TARGET_REFERENCE_POINT] as shown in the following figure. The `CUR_IMG_REF_POINT` depends on the position of the QR code. 

1. If the position encoded in the QR code is `left`:
   
   The reference point would considered to be on the right of the QR code, so that the AWS DeepRacer can pass it with the identified QR code to its left.

   ![deepracer-offroad-qr-detection-bb-left](/media/deepracer-offroad-qr-detection-bb-left.png)

1. If the position encoded is `right`:
   
   The reference point would considered to be on the left of the QR code for the AWS DeepRacer to pass it with the identified QR code to the right.

   ![deepracer-offroad-qr-detection-bb-right](/media/deepracer-offroad-qr-detection-bb-right.png)

1. When there are multiple QR codes detected (both left and right positioned)

   ![deepracer-offroad-qr-detection-bb-multipleqr](/media/deepracer-offroad-qr-detection-bb-multipleqr.png)

This delta value is published as the `DetectionDeltaMsg` data to the `/qr_detection_pkg/qr_detection_delta` topic, which is read by the AWS DeepRacer Offroad navigation node.

If no QR code is detected in three consecutive images, the `qr_detection_node` publishes a zero error (delta) signifying that the AWS DeepRacer is already at the target position and need not move, unless it encounters a QR with a limit set. 


### Action (navigation)

The AWS DeepRacer Offroad navigation ROS package creates the `deepracer_offroad_navigation_node`, which decides the action or controller message to send out based on the normalized detection error (delta) received from the `qr_detection_node`. The node uses a very simple action space to account for the various combinations of the (`x`, `y`) delta values that are expected.

#### Define the valid `DetectionDelta` values

![deepracer-offroad-navigation-moves](/media/deepracer-offroad-navigation-moves.png)

As an initial step in coming up with the action space to be used for navigation, we have to define valid detection delta ranges that will be supported by the application. The preceding diagram shows the six valid {delta_x, delta_y} ranges that can be passed to the `deepracer_offroad_navigation_node`. The `deepracer_offroad_navigation_node` is designed to work with a negative range of `delta_y` values to ensure the AWS DeepRacer device keeps moving and looking for the next waypoint code by taking forward actions when a waypoint code is detected.

These {delta_x, delta_y} values are the difference between the target position and the `CUR_IMG_REF_POINT` calculated from the QR code in the current image. The different detection delta values can be grouped into these high-level actions that can be further used to define more granular actions:

|   Case    |    Steering    |    Throttle    |
| --------- | -------------- | -------------- |
|     1     |      Left      |     Forward    |
|     2     |      NULL      |     Forward    |
|     3     |      Right     |     Forward    |
|     4     |      Left      |     NULL       |
|     5     |      NULL      |     NULL       |
|     6     |      Right     |     NULL       |

#### Define the specific action values

While implementing the navigation logic, it is important to build a safe and meaningful action space with the right granularity in steering and throttle actions. We need to map the `DetectionDelta` values passed to the `deepracer_offroad_navigation_node` to specific steering and throttle values to be sent to the AWS DeepRacer servo. These specific steering and throttle values are obtained by defining bounding brackets with different thresholds of the {delta_x, delta_y} ranges.

The granular action values and their corresponding {delta_x, delta_y} threshold values used in the application have been defined by empirically collecting the {delta_x, delta_y} value of the QR codes at different positions with respect to the AWS DeepRacer's camera. These take into account the distance of the waypoint detected (`CUR_IMG_REF_POINT`) from the current target position and the direction the AWS DeepRacer needs to turn to pass the waypoint at a safe distance. The following grid diagram shows a top-down view of the placement positions for the `CUR_IMG_REF_POINT` and target point (camera).

![deepracer-offroad-qr-detection-delta-nav-mapping](/media/deepracer-offroad-qr-detection-delta-nav-mapping.png)

These {delta_x, delta_y} values with respect to the `CUR_IMG_REF_POINT` from the camera target enable us to create a safe-distance bracket for valid actions. These brackets are then mapped to the steering and the speed values required by the AWS DeepRacer servo node. Based on the data collected, we get the following action brackets:

**Steering:**

* `DEFAULT`: No steering; when the `CUR_IMG_REF_POINT` is on the straight line of sight with respect to the camera
* `MIN_LEFT`
* `MID_LEFT`
* `MAX_LEFT`
* `MIN_RIGHT`
* `MID_RIGHT`
* `MAX_RIGHT`

**Speed:**

* `DEFAULT`: No throttle
* `FORWARD`

Every combination of the normalized delta values (`delta_x` and `delta_y`) is mapped to their corresponding brackets and a steering and throttle value is published to the servo node. Here, the navigation logic, which plans an action based on the delta combinations received from the `qr_detection_node`, can be changed to add more actions to define your own action space as best suits your use case.

Hence, using this pipeline for perception-decision-action on a loop, the AWS DeepRacer detects QR codes, plans what action is needed to bring the `CUR_IMG_REF_POINT` to the target position, and takes the action for each image it decodes, thus achieving the goal of creating a path through a series of QR codes placed on the ground.


## Demo

The following image shows a first-person view of how the AWS DeepRacer navigates through a setup path:

![deepracer-offroad-demo](/media/deepracer-offroad-demo.gif)


## Possible next steps

The AWS DeepRacer Offroad project is an example of how the individual nodes in the Follow the Leader (FTL) sample project were modified to achieve a different goal. The `object_detection_node` has been modified to replace the machine learning inference step with an object-detection model with a simple Python wrapper for QR code detection. Also, with few tweaks to the `ftl_navigation_node`, we implement the `deepracer_offroad_navigation_node`, which handles the action planning based on detection deltas from the `qr_detection_node`.

This way, the individual nodes used in the AWS DeepRacer Offroad sample project or the entire sample project can be used to develop a new feature or racing concept using QR codes.

To learn more, experiment with the following ideas:

* **Find your way out**: Set up the QR codes as a maze and encode instructions in them to let an AWS DeepRacer decide the path it wants to take.
* **Record and replay**: Add a memory store to save actions from the initial identification of a special waypoint type and rerun them when encountering the same waypoint type.
* **Build your own track and race!**


## Summary

The AWS DeepRacer Offroad sample project leverages most of the concepts used in the AWS DeepRacer application. To learn more about the AWS DeepRacer core application, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).


## Resources

* [Getting started with AWS DeepRacer OpeSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Create QR codes to set up an Offroad path](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/create-qrcodes-to-setup-offroad-path.md)
