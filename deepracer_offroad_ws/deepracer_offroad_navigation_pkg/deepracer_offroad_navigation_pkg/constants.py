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

ACTION_PUBLISH_TOPIC = "offroad_drive"
SET_MAX_SPEED_SERVICE_NAME = "set_max_speed"

QR_DETECTION_PKG_NS = "/qr_detection_pkg"
QR_DETECTION_DELTA_TOPIC = f"{QR_DETECTION_PKG_NS}/qr_detection_delta"


class DeltaValueMap():
    """Class with the delta values mapping to action brackets
    """
    FORWARD_DELTA_Y = 0.0
    FORWARD_RIGHT_DELTA_X = 0.03
    FORWARD_MID_RIGHT_DELTA_X = 0.08
    FORWARD_MAX_RIGHT_DELTA_X = 1.0
    FORWARD_LEFT_DELTA_X = -0.03
    FORWARD_MID_LEFT_DELTA_X = -0.08
    FORWARD_MAX_LEFT_DELTA_X = -1.0


class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    FORWARD = 0.3
    REVERSE = -0.3
    MAX_LEFT = 1.0
    MID_LEFT = 0.4
    MIN_LEFT = 0.2
    MAX_RIGHT = -1.0
    MID_RIGHT = -0.4
    MIN_RIGHT = -0.2
    DEFAULT = 0.0


class ActionSpaceConfig():
    """Class with the action space configuration which represent the
       angle and throttle with respect to different action categories.
    """
    NO_ACTION = (ActionValues.DEFAULT, ActionValues.DEFAULT)
    FORWARD = (ActionValues.DEFAULT, ActionValues.FORWARD)
    MIN_LEFT_FORWARD = (ActionValues.MIN_LEFT, ActionValues.FORWARD)
    MID_LEFT_FORWARD = (ActionValues.MID_LEFT, ActionValues.FORWARD)
    MAX_LEFT_FORWARD = (ActionValues.MAX_LEFT, ActionValues.FORWARD)
    MIN_RIGHT_FORWARD = (ActionValues.MIN_RIGHT, ActionValues.FORWARD)
    MID_RIGHT_FORWARD = (ActionValues.MID_RIGHT, ActionValues.FORWARD)
    MAX_RIGHT_FORWARD = (ActionValues.MAX_RIGHT, ActionValues.FORWARD)


# Max speed percentage on a scale between 0.0 and 1.0.
# The maximum speed value is used to non linearly map the raw value obtained for the forward
# and reverse throttle to the PWM values of the servo/motor.
# We use the maximum speed % to map to a range of [1.0, 5.0] speed scale values using the
# calculated coefficients of the equation y = ax^2 + bx.
# This allows us to recalculate the curve for each maximum speed % value and use that to
# map the speed values. The idea behind this mapping is a lower percentage of maximum speed %
# should map to a higher speed scale value while calculating the coefficients so that the curve
# is more flatter and the impact of actual speed values is less for lower max speed %.
MAX_SPEED_PCT = 0.68

# Action space mapped to on the vehicle for speed values of 0.8 and 0.4.
DEFAULT_SPEED_SCALES = [1.0, 0.8]
# Speed scale bounds to pick from while calculating the coefficients.
MANUAL_SPEED_SCALE_BOUNDS = [1.0, 5.0]
# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.08
