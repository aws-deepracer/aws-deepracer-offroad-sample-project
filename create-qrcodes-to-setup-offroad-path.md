# Create QR Codes to setup Offroad path

## Overview

The DeepRacer Offroad sample project is a sample application built on top of the existing AWS DeepRacer core application. It uses QR Code detection to identify and navigate through a path setup using QR Codes to guide the AWS DeepRacer device. For detailed information on DeepRacer Offroad sample project, see DeepRacer Offroad sample project [Getting Started](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md) section.

The DeepRacer Offroad project requires a series of QR Codes to be setup on the floor in order to create an “Offroad” track for the DeepRacer to drive through them.

## Steps to create the QR Codes:

1. **Install qrcode package:** The encoding is done by QR code generator python wrapper “qrcode” (https://pypi.org/project/qrcode/) to create different waypoints codes with information encoded in the specific format required. Install [Pure python QR Code generator](https://pypi.org/project/qrcode/)

        pip3 install qrcode[pil]

1. **Create QR codes:** A set of code snippets to generate example QR codes are added in the sample python script “qr_generation_script.py”. This can be used as a template to generate your own QR codes.

        import qrcode

        # QR positioned on left of the DeepRacer
        qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=200,
        border=1,
        )
        qr.add_data('DR: {"wp": 1, "p": "l"}')
        img = qr.make_image(fill_color="black", back_color="white")
        img.save("deepracer_offroad_qr_images/left_wp1.png")

### Encoding information in the QR Codes:

The data required by the qr_detection_node to parse the information in the Offroad application should be in a specific format as shown below:

        'DR: {"wp": 1, "p": "l"}'

OR

        'DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [0.0, -0.2], "limit": 35}'

The convention is to encode a string in the format ‘DR: waypoint_json’, where the waypoint_json can contain the JSON Fields that currently supported in qr_detection_node:

* **Waypoint number (Required):**
    * Key: **wp**
    * Values: 
        * Positive Integers
        * Waypoint number of the QR Code

* **Position of the QR code (Required):**
    * Key: **p**
    * Values: 
        * Strings “l” / “r”
        * Position of the QR Code (Left/Right of the DeepRacer)

* **Type of the waypoint (Optional):**
    * Key: **type**
    * Values: 
        * Strings “action_start”/“action_stop”
        * action_start: Sets the ongoing_action in qr_detection_node to the encoded delta
        * action_stop: Clears the ongoing_action in qr_detection_node and sets it to None

*  **DetectionDelta values to be published (Optional):**
    * Key: **delta**
    * Values:
        * List of floats [x,y]
        * Delta values pairs (x,y) to be passed to navigation when the QR Code has a waypoint type value “action_start” encoded.

*  **Maximum limit value for the ongoing action (Optional):**
    * Key: **limit**
    * Values: 
        * Positive Integers [1, infinity]
        * Sets the limit counter to run a specific ongoing action for decoded number of frames when QR Code has a “type” encoded.


## Setting up the QR Codes on the ground

Once you have the necessary QR Codes encoded with the JSON for your use case printed full size on a A4 sized sheet, place them such that you create a path with a combination of multiple QR Codes as shown in the sample path below:

![deepracer-offroad-sample-path](/media/deepracer-offroad-sample-path.png)

In the example shown above, there are three basic left positioned waypoints and four right positioned waypoints. The QR Codes are placed alternately in a left-right sequence as seen in the image above, providing ample space for the DeepRacer to move through the path created and also ensuring they are not placed too far from each other. The setup of the QR Codes will decide how well the Deepracer is able to navigate through the path created for it.


## Visualize the path planned by DeepRacer w.r.t QR placement:

Although the performance of the DeepRacer to detect and decode QR Codes, and pave a way through a series of QR codes can vary with change in speed of the car, size and quality of the QR code, etc, there is a simple way to verify if the current placement of the QR codes creates a path that the DeepRacer can follow.

The image stream output in the qr_detection_node after QR Code detection has additional visualization added on top that can be used to correct the waypoint code placements. The output images are streamed through web_video_server which can be accessed by opening up a browser on the DeepRacer device and navigating to localhost:8080 to view the visualization which looks like the following:

![deepracer-offroad-visualization](/media/deepracer-offroad-visualization.png)

Looking at the images above, the pink line connects the REFERENCE POINTS calculated from each QR Detected while the green line shows the actual path which the DeepRacer is planning to take. As the DeepRacer moves, the planned path will change depending on the detected waypoints. This helps to verify if the waypoint codes are placed correctly and in a way that the DeepRacer is able to detect the QR Codes, decode the content and plan the appropriate action which you envision for the DeepRacer to take. 

## Resources

* Pure python QR Code generator: [https://pypi.org/project/qrcode](https://pypi.org/project/qrcode)
* DeepRacer Offroad sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)



