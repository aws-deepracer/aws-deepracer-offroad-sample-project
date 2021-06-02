# Create QR codes to set up the Offroad path

## Overview

The AWS DeepRacer Offroad sample project is a sample application built on top of the existing AWS DeepRacer core application. It uses QR code detection to identify and navigate through a path set up using QR codes to guide the AWS DeepRacer device. For detailed information on the AWS DeepRacer Offroad sample project, see the AWS DeepRacer Offroad sample project [Getting started](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md) section.

The AWS DeepRacer Offroad project requires a series of QR codes to be set up on the floor in order to create an “Offroad” track for the DeepRacer to navigate.

## Steps to create the QR codes

1. **Install the qrcode package:** A [`qrcode` QR code generator Python wrapper](https://pypi.org/project/qrcode/) creates different waypoint codes with information encoded in the specific format required. Install the [Pure python QR Code generator](https://pypi.org/project/qrcode/):

        pip3 install qrcode[pil]

1. **Create QR codes:** The `qr_generation_script.py` sample Python script contains a set of code snippets to generate example QR codes. You can use this as a template to generate your own QR codes.

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

### Encoding information in the QR codes

The data required by the `qr_detection_node` to parse the information in the Offroad application should be in one of the two following specific formats:

   *     'DR: {"wp": 1, "p": "l"}'



   *     'DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [0.0, -0.2], "limit": 35}'

The convention is to encode a string in the format `DR: waypoint_json`, where the `waypoint_json` can contain the JSON fields currently supported in `qr_detection_node`.

* **Waypoint number (required):**
    * Key: **wp**
    * Values: 
        * Positive integers
        * Waypoint number of the QR code

* **Position of the QR code (Required):**
    * Key: `p`
    * Values: 
        * Strings `l` / `r`
        * Position of the QR code (left or right of the AWS DeepRacer)

* **Type of the waypoint (optional):**
    * Key: `type`
    * Values: 
        * Strings `action_start`/`action_stop`
        * `action_start`: Sets the `ongoing_action` in `qr_detection_node` to the encoded delta
        * `action_stop`: Clears the `ongoing_action` in `qr_detection_node` and sets it to `None`

*  **DetectionDelta values to be published (optional):**
    * Key: `delta`
    * Values:
        * List of floats [x,y]
        * Delta values pairs (`x`,`y`) to be passed to navigation when the QR code has an `action_start` waypoint type value encoded.

*  **Maximum limit value for the ongoing action (optional):**
    * Key: `limit`
    * Values: 
        * Positive Integers [1, infinity]
        * Sets the limit counter to run a specific ongoing action for decoded number of frames when the QR code has a `type` encoded.


## Setting up the QR codes on the ground

Once you have the necessary QR codes encoded with the JSON for your use case printed full size on a A4-sized sheet, place them to create a path with a combination of multiple QR codes as shown in the following sample path:

![deepracer-offroad-sample-path](/media/deepracer-offroad-sample-path.png)

In the preceding example, there are three basic left-positioned waypoints and four right-positioned waypoints. The QR codes are placed alternately in a left-right sequence as shown in the preceding image, providing ample space for the AWS DeepRacer to move through the path and also ensuring they are not placed too far from each other. The setup of the QR codes decides how well the AWS Deepracer is able to navigate through the path created for it.


## Visualize the path planned by the AWS DeepRacer using the QR placement

Although the performance of the AWS DeepRacer to detect and decode QR codes and pave a way through a series of QR codes is affected by variables such as changes in the speed of the car and the size and quality of the QR codes, there is a simple way to verify if the current placement of the QR codes creates a path that the AWS DeepRacer can follow.

The image stream output in the `qr_detection_node` after QR code detection has an additional visualization added on top that can be used to correct the waypoint code placements. The output images are streamed through `web_video_server`, which can be accessed by opening a browser on the AWS DeepRacer device and navigating to `localhost:8080` to view the visualization. The visualization should resemble the following example: 

![deepracer-offroad-visualization](/media/deepracer-offroad-visualization.png)

In the preceding image, the pink line connects the `REFERENCE POINTS` calculated from each QR code detected, while the green line shows the actual path the AWS DeepRacer is planning to take. As the AWS DeepRacer moves, the planned path changes depending on the detected waypoints. This helps to verify if the waypoint codes are placed correctly and that the AWS DeepRacer is able to detect the QR codes, decode the content, and plan the appropriate action. 

## Resources

* [Pure Python QR code generator](https://pypi.org/project/qrcode)
* [Getting started with the AWS DeepRacer Offroad sample project](https://github.com/aws-deepracer/aws-deepracer-offroad-sample-project/blob/main/getting-started.md)



