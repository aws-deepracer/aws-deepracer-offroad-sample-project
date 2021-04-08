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

"""
deepracer_offroad_navigation_node.py

This module decides the action messages (servo control messages specifically angle
and throttle) to be sent out using the detection deltas from qr_detection_node.

The node defines:
    detection_delta_subscriber: A subscriber to the /qr_detection_pkg/qr_detection_delta
                                published by the qr_detection_pkg with the normalized delta
                                of the detected QR code position from the target (reference)
                                position with respect to x and y axes.
    The node defines:
    action_publisher: A publisher to publish the action (angle and throttle values).
    set_max_speed_service: A service to dynamically set MAX_SPEED_PCT representing
                           the max speed percentage scale as per request.
"""
import time
import signal
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)

from deepracer_interfaces_pkg.msg import (DetectionDeltaMsg,
                                          ServoCtrlMsg)
from deepracer_interfaces_pkg.srv import SetMaxSpeedSrv
from deepracer_offroad_navigation_pkg import (constants,
                                              utils)


class DeepracerOffroadNavigationNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and throttle) to be sent out using the detection deltas from qr_detection_node.
    """

    def __init__(self, qos_profile):
        """Create a DeepracerOffroadNavigationNode.
        """
        super().__init__('deepracer_offroad_navigation_node')
        self.get_logger().info("deepracer_offroad_navigation_node started.")
        self.current_waypoint = 0

        # Double buffer to hold the input deltas in x and y from qr_detection_node.
        self.delta_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Create subscription to detection deltas from qr_detection_node.
        self.detection_delta_subscriber = \
            self.create_subscription(DetectionDeltaMsg,
                                     constants.QR_DETECTION_DELTA_TOPIC,
                                     self.detection_delta_cb,
                                     qos_profile)

        # Creating publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(ServoCtrlMsg,
                                                      constants.ACTION_PUBLISH_TOPIC,
                                                      qos_profile)

        # Service to dynamically set MAX_SPEED_PCT.
        self.set_max_speed_service = self.create_service(SetMaxSpeedSrv,
                                                         constants.SET_MAX_SPEED_SERVICE_NAME,
                                                         self.set_max_speed_cb)

        # Initializing the msg to be published.
        msg = ServoCtrlMsg()
        msg.angle, msg.throttle = constants.ActionValues.DEFAULT, constants.ActionValues.DEFAULT

        self.lock = threading.Lock()
        # Default maximum speed percentage (updated as per request using service call).
        self.max_speed_pct = constants.MAX_SPEED_PCT

        # Create a background servo publish thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.action_publish, args=(msg,))
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input delta: {constants.QR_DETECTION_DELTA_TOPIC}")

    def wait_for_thread(self):
        """Function which joins the created background thread.
        """
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread.
        """
        self.stop_thread = True

    def set_max_speed_cb(self, req, res):
        """Callback which dynamically sets the max_speed_pct.

        Args:
            req (SetMaxSpeedSrv.Request): Request object with the updated
                                                  max speed percentage.
            res (SetMaxSpeedSrv.Response): Response object with error(int) flag
                                                   indicating successful max speed pct
                                                   update.

        Returns:
            SetMaxSpeedSrv.Response: Response object with error(int) flag indicating
                                             successful max speed pct update.

        """
        with self.lock:
            try:
                self.max_speed_pct = req.max_speed_pct
                self.get_logger().info(f"Incoming request: max_speed_pct: {req.max_speed_pct}")
                res.error = 0
            except Exception as ex:
                self.get_logger().error(f"Failed set max speed pct: {ex}")
                res.error = 1
        return res

    def detection_delta_cb(self, detection_delta):
        """Call back for whenever detection delta for a perception
           is received from qr_detection_node.

        Args:
            detection_delta (DetectionDeltaMsg): Message containing the normalized detection
                                                       delta in x and y axes respectively passed as
                                                       a list.
        """

        self.delta_buffer.put(detection_delta)

    def plan_action(self, delta):
        """Helper method to calculate action to be undertaken from the detection delta
           received from qr_detection_node. This has been modified from the
           Follow the Leader Navigation Node in Follow the Leader sample project to provide
           more action brackets for Left/Right movement based on the detection deltas.
           Follow the Leader sample project:
           https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project
           Also the action space for reverse has been removed since its no longer required
           for the DeepRacer Off-road sample project.

        Args:
        delta (list of floats): detection deltas in x and y axes respectively.

        Returns:
        (tuple of floats): Action Space Configuration defined in constants.py
        """
        delta_x = delta[0]
        delta_y = delta[1]

        if delta_y < constants.DeltaValueMap.FORWARD_DELTA_Y:
            # Forward Bracket
            if delta_x < constants.DeltaValueMap.FORWARD_LEFT_DELTA_X \
                    and delta_x > constants.DeltaValueMap.FORWARD_MID_LEFT_DELTA_X:
                # Min Left
                self.get_logger().info("MIN_LEFT_FORWARD")
                return constants.ActionSpaceConfig.MIN_LEFT_FORWARD
            elif delta_x <= constants.DeltaValueMap.FORWARD_MID_LEFT_DELTA_X \
                    and delta_x > constants.DeltaValueMap.FORWARD_MAX_LEFT_DELTA_X:
                # Mid Left
                self.get_logger().info("MID_LEFT_FORWARD")
                return constants.ActionSpaceConfig.MID_LEFT_FORWARD
            elif delta_x <= constants.DeltaValueMap.FORWARD_MAX_LEFT_DELTA_X:
                # Max Left
                self.get_logger().info("MAX_LEFT_FORWARD")
                return constants.ActionSpaceConfig.MAX_LEFT_FORWARD
            elif delta_x > constants.DeltaValueMap.FORWARD_RIGHT_DELTA_X \
                    and delta_x < constants.DeltaValueMap.FORWARD_MID_RIGHT_DELTA_X:
                # Min Right
                self.get_logger().info("MIN_RIGHT_FORWARD")
                return constants.ActionSpaceConfig.MIN_RIGHT_FORWARD
            elif delta_x >= constants.DeltaValueMap.FORWARD_MID_RIGHT_DELTA_X \
                    and delta_x < constants.DeltaValueMap.FORWARD_MAX_RIGHT_DELTA_X:
                # Mid Right
                self.get_logger().info("MID_RIGHT_FORWARD")
                return constants.ActionSpaceConfig.MID_RIGHT_FORWARD
            elif delta_x >= constants.DeltaValueMap.FORWARD_MAX_RIGHT_DELTA_X:
                # Max Right
                self.get_logger().info("MAX_RIGHT_FORWARD")
                return constants.ActionSpaceConfig.MAX_RIGHT_FORWARD
            else:
                # No steering
                self.get_logger().info("FORWARD")
                return constants.ActionSpaceConfig.FORWARD

        else:
            # No Action
            self.get_logger().info("NO_ACTION")
            return constants.ActionSpaceConfig.NO_ACTION

    def get_mapped_action(self, action_config, max_speed_pct):
        """Return the angle and throttle values to be published for servo.

        Args:
            action_config (tuple of floats): Tuple with the Angle and Throttle
                                             from the planned action.
            max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                   from maximum speed input.
        Returns:
            angle (float): Angle value to be published to servo.
            throttle (float): Throttle value to be published to servo.
        """
        angle, categorized_throttle = action_config
        throttle = self.get_rescaled_manual_speed(categorized_throttle, max_speed_pct)
        return angle, throttle

    def get_rescaled_manual_speed(self, categorized_throttle, max_speed_pct):
        """Return the non linearly rescaled speed value based on the max_speed_pct.

        Args:
            categorized_throttle (float): Float value ranging from -1.0 to 1.0.
            max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                   from maximum speed input.
        Returns:
            float: Categorized value of the input speed.
        """
        # return 0.0 if categorized_throttle or maximum speed pct is 0.0.
        if categorized_throttle == 0.0 or max_speed_pct == 0.0:
            return 0.0

        # get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
        # The lower the update_speed_scale_value parameter, higher the impact on the
        # final mapped_speed.
        # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
        # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
        # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
        #     max_speed_pct = 1.0; update_speed_scale_value = 1
        # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
        # higher possible values.
        #   Example: update_speed_scale_value = 1.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
        # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
        # lower possible values.
        #   Example: update_speed_scale_value = 3.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

        inverse_max_speed_pct = (1 - max_speed_pct)

        update_speed_scale_value = \
            constants.MANUAL_SPEED_SCALE_BOUNDS[0] + \
            inverse_max_speed_pct * \
            (constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0])

        if update_speed_scale_value < 0.0:
            self.get_logger().info("The update_speed_scale_value is negative, taking absolute value.")
            update_speed_scale_value = abs(update_speed_scale_value)
        speed_mapping_coefficients = dict()

        # recreate the mapping coefficients for the non-linear equation ax^2 + bx based on
        # the update_speed_scale_value.
        # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
        # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
        speed_mapping_coefficients["a"] = \
            (1.0 / update_speed_scale_value**2) * \
            (2.0 * constants.DEFAULT_SPEED_SCALES[0] - 4.0 * constants.DEFAULT_SPEED_SCALES[1])
        speed_mapping_coefficients["b"] = \
            (1.0 / update_speed_scale_value) * \
            (4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0])
        return math.copysign(1.0, categorized_throttle) * \
            (speed_mapping_coefficients["a"] * abs(categorized_throttle)**2 +
             speed_mapping_coefficients["b"] * abs(categorized_throttle))

    def action_publish(self, msg):
        """Function which runs in a separate thread to read qr detection delta
           from double buffer, decides the action and sends it to servo.

        Args:
            msg: detection_delta (DetectionDeltaMsg): Message containing the normalized
                 detection delta in x and y axes respectively passed as a list.
        """
        try:
            while not self.stop_thread:
                # Get a new message to plan action on
                detection_delta = self.delta_buffer.get()
                action_config = self.plan_action(detection_delta.delta)
                msg.angle, msg.throttle = self.get_mapped_action(action_config,
                                                                 self.max_speed_pct)
                # Publish msg based on action planned and mapped from a new qr detection.
                self.action_publisher.publish(msg)
                time.sleep(constants.DEFAULT_SLEEP)

        except Exception as ex:
            self.get_logger().error(f"Failed to publish action to servo: {ex}")
            # Stop the car
            msg.angle, msg.throttle = constants.ActionValues.DEFAULT, constants.ActionValues.DEFAULT
            self.action_publisher.publish(msg)
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        deepracer_offroad_navigation_node = DeepracerOffroadNavigationNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            deepracer_offroad_navigation_node.get_logger().info("Signal Handler initiated")
            deepracer_offroad_navigation_node.thread_shutdown()
            deepracer_offroad_navigation_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(deepracer_offroad_navigation_node, executor)
    except Exception as ex:
        deepracer_offroad_navigation_node.get_logger().error(f"Exception in Deepracer Offroad Navigation Node: {ex}")
        deepracer_offroad_navigation_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    deepracer_offroad_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
