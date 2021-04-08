#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

DELTA_PUBLISHER_TOPIC = "qr_detection_delta"
DISPLAY_IMAGE_PUBLISHER_TOPIC = "qr_detection_display"

SENSOR_FUSION_PKG_NS = "/sensor_fusion_pkg"
SENSOR_FUSION_TOPIC = f"{SENSOR_FUSION_PKG_NS}/sensor_msg"

# The X and Y Margins which will be added during the calculation
# of detection delta since we want the DeepRacer to pass the QR Code
# at a certain safe distance using the calculated reference point.
X_MARGIN = 100
Y_MARGIN = 30

# Threshold values for the delta_y to calculate valid/invalid waypoints
DELTA_DISTANCE_THRESHOLD_Y = 0.25
DELTA_SWTICH_DISTANCE_THRESHOLD_Y = 0.40

# Input image size [Resolution of camera]
WIDTH = 640
HEIGHT = 480


class CVConstants():
    """ Class with the constants required for adding visualization
    to image.
    """
    GREEN = (232, 35, 244)
    BLUE = (232, 35, 33)
    LIGHT_GREEN = (34, 232, 33)


class QRKeys():
    """Class with constant keys for QR and waypoints
    """
    DEEPRACER = "DR:"
    WAYPOINT = "wp"
    POSITION = "p"
    TYPE = "type"
    DELTA = "delta"
    LIMIT = "limit"
    REFERENCE = "ref"
    BOUNDING_BOX = "bb"


class QRValues():
    """Class with the constant values for QR Code
    """
    LEFT = "l"
    RIGHT = "r"
    ACTION_START = "action_start"
    ACTION_STOP = "action_stop"
