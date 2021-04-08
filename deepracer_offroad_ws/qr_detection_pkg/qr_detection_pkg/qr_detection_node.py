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
qr_detection_node.py

This module creates the qr_detection_node which is responsible for collecting
sensor data (camera images) from sensor_fusion_pkg and running qr code detection,
providing normalized delta from target for deepracer_offroad_navigation_pkg.

The node defines:
    image_subscriber: A subscriber to the /sensor_fusion_pkg/sensor_msg published
                      by the sensor_fusion_pkg with sensor data.
    display_image_publisher: A publisher to publish the Image message using
                             web_video_server.
    qr_delta_publisher: A publisher to publish the normalized error (delta) of the
                        detected qr code from the target (reference) position
                        with respect to x and y axes.
"""
import signal
import threading
import cv2
import numpy as np
import json
from pyzbar import pyzbar
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from deepracer_interfaces_pkg.msg import (EvoSensorMsg,
                                          DetectionDeltaMsg)
from qr_detection_pkg import (constants,
                              utils)


class QRDetectionNode(Node):
    """Node responsible for collecting sensor data (camera images) from sensor_fusion_pkg
       and and running qr code detection, providing normalized delta from target for
       deepracer_offroad_navigation_pkg.
    """

    def __init__(self, qos_profile):
        """Create a QRDetectionNode.
        """
        super().__init__('qr_detection_node')
        self.get_logger().info("qr_detection_node started.")
        self.detect_waypoint_number = 1
        # Double buffer to hold the input images for inference.
        self.input_buffer = utils.DoubleBuffer(clear_data_on_get=True)
        # Calculate target position to track.
        self.target_x, self.target_y = self.calculate_target_reference_point(constants.WIDTH,
                                                                             constants.HEIGHT)
        # Create subscription to sensor messages from camera.
        self.image_subscriber = self.create_subscription(EvoSensorMsg,
                                                         constants.SENSOR_FUSION_TOPIC,
                                                         self.on_image_received_cb,
                                                         qos_profile)
        # Creating publisher for display_image.
        self.display_image_publisher = \
            self.create_publisher(Image,
                                  constants.DISPLAY_IMAGE_PUBLISHER_TOPIC,
                                  10)
        # Creating publisher for error (delta) from target bb position.
        self.delta_publisher = self.create_publisher(DetectionDeltaMsg,
                                                     constants.DELTA_PUBLISHER_TOPIC,
                                                     qos_profile)
        self.bridge = CvBridge()
        self.last_three_waypoints = list()
        # Keep track of the action running.
        self.ongoing_action = None
        # Keep a limit counter to run a specific ongoing action for decoded number of frames.
        self.limit = 1
        # Launching a separate thread to run inference.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.run_waypoint_detection)
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input images on {constants.SENSOR_FUSION_TOPIC}")

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

    def on_image_received_cb(self, sensor_data):
        """Call back for adding to the input double buffer whenever
           new sensor image is received from sensor_fusion_node.

        Args:
            sensor_data (EvoSensorMsg): Message containing sensor images and lidar data.
        """
        self.input_buffer.put(sensor_data)

    def preprocess(self, sensor_data):
        """Method that preprocesses the input data to be provided for inference to network.

        Args:
            sensor_data (EvoSensorMsg): Contains sensor images and lidar data.

        Returns:
            image: Preprosessed image expected.
        """
        image = self.bridge.imgmsg_to_cv2(sensor_data.images[0])
        return image

    def calculate_target_reference_point(self, image_width, image_height):
        """Method that calculates the target's x and y co-ordinates for
           bounding box to be used as reference.

        Args:
            image_width (int): Width of the preprocessed image.
            image_height (int): Height of the preprocessed image.

        Returns:
            target_x, target_y (float)
        """
        target_x = float(image_width) / 2.0
        target_y = float(image_height) - (float(image_height) / 3.0)
        self.get_logger().info(f"Target Center: x={target_x} y={target_y}")
        return target_x, target_y

    def calculate_cur_img_ref_point(self,
                                    top_left_x,
                                    top_left_y,
                                    bottom_right_x,
                                    bottom_right_y,
                                    qr_code_placement_loc=constants.QRValues.LEFT):
        """Method that calculates the current image's reference point x and y co-ordinates
           representing to calculate the delta values.

        Args:
            top_left_x (int)
            top_left_y (int)
            bottom_right_x (int)
            bottom_right_y (int)
            qr_code_placement_loc(str): Placement location read from the qr code. Defaults to left.

        Returns:
            ref_x, ref_y (float): Containing the x and y coordinates of
                                  reference point to be used for delta calculation.
        """
        if qr_code_placement_loc == constants.QRValues.LEFT:
            ref_x = bottom_right_x + constants.X_MARGIN
            ref_y = bottom_right_y
        elif qr_code_placement_loc == constants.QRValues.RIGHT:
            ref_x = top_left_x - constants.X_MARGIN
            ref_y = bottom_right_y
        else:
            raise Exception("Invalid QR Code placement location detected.")
        return ref_x, ref_y

    def calculate_delta(self, target_x, target_y, ref_x, ref_y):
        """Method that calculates the normalized error (delta) of the
           detected qr code from the target (reference) position
           with respect to x and y axes.

        Args:
            target_x (float): Target x co-ordinate.
            target_y (float): Target y co-ordinate.
            ref_x (float): x co-ordinate of reference point.
            ref_y (float): y co-ordinate of reference point.

        Returns:
            delta (DetectionDeltaMsg): Normalized Error (delta) in x and y respectively
            returned as a list of floats and converted to DetectionDeltaMsg.
        """
        try:
            delta_x = (ref_x - target_x) / constants.WIDTH
            delta_y = (ref_y - target_y) / constants.HEIGHT
            delta = DetectionDeltaMsg()
            delta.delta = [delta_x, delta_y]
            self.get_logger().info(f"Delta from target position: {delta_x} {delta_y}")
            return delta
        except Exception as ex:
            self.get_logger().error(f"Failed to calculate delta: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def parse_qr_json(self, qrcode_info):
        """Method that parses the QR info and returns the
        JSON dictionary.

        Args:
            qrcode_info (String): Decoded String for QR Code.

        Returns:
            qr_json_detected (Dictionary): JSON Dictionary with encoded data in QR Code

        """
        qr_json_detected = json.loads(qrcode_info.split(constants.QRKeys.DEEPRACER)[1].strip())
        self.get_logger().debug(f"JSON Detected {qr_json_detected}")
        # Check if the QR Code has specific action encoded in "type"
        # and add it to waypoint_type.
        if constants.QRKeys.TYPE in qr_json_detected:
            waypoint_type = qr_json_detected[constants.QRKeys.TYPE]
            # Check if detected QR has a new action delta encoded
            # which will set the ongoing_action.
            if waypoint_type == constants.QRValues.ACTION_START:
                # Get the encoded delta values for the specific action start
                valid_detection_delta = DetectionDeltaMsg()
                valid_detection_delta.delta = qr_json_detected[constants.QRKeys.DELTA]
                # Set the ongoing action delta to detected action delta
                self.ongoing_action = valid_detection_delta
                # Check if a limit is encoded in QR code along with the action delta
                # which decides the number of frames the action delta to be published.
                if constants.QRKeys.LIMIT in qr_json_detected:
                    # Set the limit value to encoded limit value
                    self.limit = qr_json_detected[constants.QRKeys.LIMIT]
                else:
                    # Set the limit value to infinite if no limit value is encoded
                    # ie; keep running the action until we see an action_stop QR
                    self.limit = float('inf')
            elif waypoint_type == constants.QRValues.ACTION_STOP:
                # Clear the ongoing_action and set it to valid_detection_delta (None)
                valid_detection_delta = None
                self.ongoing_action = valid_detection_delta

        return qr_json_detected

    def decode_qr(self, image):
        """Method that decodes the QR from input image and returns the
        waypoints as a dictionary.

        Args:
            image (Image): Input image to scan for QR codes.

        Returns:
            waypoints (Dictionary): Consisting of the waypoints decoded else returns an empty dict.
        """
        try:
            waypoints = dict()
            # Decrement the limit
            self.limit = max(self.limit - 1, 0)
            # Check if limit has been reached to stop an ongoing action delta.
            if self.limit == 0:
                self.ongoing_action = None
                self.limit = 1
            # Decode qrcodes using pyzbar
            qrcodes = pyzbar.decode(image)

            for qrcode in qrcodes:
                x, y, w, h = qrcode.rect
                # Decode the string in QR Code
                qrcode_info = qrcode.data.decode('utf-8')
                # coordinates of the top left bounding box corner.
                top_left_x = np.int(x)
                top_left_y = np.int(y)
                # coordinates of the bottom right bounding box corner.
                bottom_right_x = np.int(x + w)
                bottom_right_y = np.int(y + h)

                if constants.QRKeys.DEEPRACER in qrcode_info:
                    # Default waypoint type
                    waypoint_type = constants.QRKeys.WAYPOINT
                    qr_json_detected = self.parse_qr_json(qrcode_info)
                    ref_x, ref_y = \
                        self.calculate_cur_img_ref_point(top_left_x,
                                                         top_left_y,
                                                         bottom_right_x,
                                                         bottom_right_y,
                                                         qr_code_placement_loc=\
                                                         qr_json_detected[constants.QRKeys.POSITION])
                    waypoints[qr_json_detected[constants.QRKeys.WAYPOINT]] = \
                        {
                            constants.QRKeys.WAYPOINT: qr_json_detected[constants.QRKeys.WAYPOINT],
                            constants.QRKeys.POSITION: qr_json_detected[constants.QRKeys.POSITION],
                            constants.QRKeys.REFERENCE: [ref_x, ref_y],
                            constants.QRKeys.BOUNDING_BOX: [top_left_x,
                                                            top_left_y,
                                                            bottom_right_x,
                                                            bottom_right_y],
                            constants.QRKeys.TYPE: waypoint_type
                    }
                    self.get_logger().info(f"Detected waypoints: {waypoints}")
            return waypoints
        except Exception as ex:
            self.get_logger().error(f"Failed decode qr step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def publish_qr_detection_visualization(self,
                                           display_image,
                                           valid_detection_point,
                                           valid_waypoints,
                                           invalid_waypoints):
        """Method to add visualization on top of received image
        after inference.

        Args:
            display_image (Image): The input image.
            valid_detection_point (list): List of valid detection points.
            valid_waypoints (list): List of valid waypoints.
            invalid_waypoints (list): List of invalid waypoints.
        """
        try:
            # Add visualization for valid waypoints
            if len(valid_waypoints) > 0:
                waypoint_connection_path_points = []
                for valid_waypoint in valid_waypoints:
                    valid_waypoint_ref = valid_waypoint[constants.QRKeys.REFERENCE]
                    valid_waypoint_bb = valid_waypoint[constants.QRKeys.BOUNDING_BOX]
                    valid_waypoint_num = valid_waypoint[constants.QRKeys.WAYPOINT]
                    valid_waypoint_pos = valid_waypoint[constants.QRKeys.POSITION]

                    # Drawing bounding boxes with green on the image for valid waypoint.
                    cv2.rectangle(display_image,
                                  (valid_waypoint_bb[0], valid_waypoint_bb[1]),
                                  (valid_waypoint_bb[2], valid_waypoint_bb[3]),
                                  constants.CVConstants.GREEN,
                                  2)
                    # Printing reference center on the image for valid waypoint.
                    cv2.circle(display_image,
                               (int(valid_waypoint_ref[0]),
                                int(valid_waypoint_ref[1])),
                               5,
                               constants.CVConstants.GREEN,
                               -1)
                    cv2.putText(display_image,
                                f"Valid: {valid_waypoint_num}; Pos: {valid_waypoint_pos}",
                                (int(valid_waypoint_bb[0]), int(valid_waypoint_bb[3]) + 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                constants.CVConstants.GREEN,
                                3)
                    waypoint_connection_path_points.append((int(valid_waypoint_ref[0]),
                                                            int(valid_waypoint_ref[1])))
                # Draw path points
                start_point = (int(self.target_x),
                               int(self.target_y))
                for end_point in waypoint_connection_path_points:
                    cv2.line(display_image,
                             start_point,
                             end_point,
                             constants.CVConstants.GREEN,
                             2,
                             cv2.LINE_8)
                    start_point = end_point

            # Add visualization for invalid waypoints
            if len(invalid_waypoints) > 0:
                for invalid_waypoint in invalid_waypoints:
                    invalid_waypoint_ref = invalid_waypoint[constants.QRKeys.REFERENCE]
                    invalid_waypoint_bb = invalid_waypoint[constants.QRKeys.BOUNDING_BOX]
                    invalid_waypoint_num = invalid_waypoint[constants.QRKeys.WAYPOINT]
                    invalid_waypoint_pos = invalid_waypoint[constants.QRKeys.POSITION]
                    # Drawing bounding boxes with blue on the image for invalid waypoint.
                    cv2.rectangle(display_image,
                                  (invalid_waypoint_bb[0], invalid_waypoint_bb[1]),
                                  (invalid_waypoint_bb[2], invalid_waypoint_bb[3]),
                                  constants.CVConstants.BLUE,
                                  2)
                    # Printing reference center on the image for invalid waypoint.
                    cv2.circle(display_image,
                               (int(invalid_waypoint_ref[0]),
                                int(invalid_waypoint_ref[1])),
                               5,
                               constants.CVConstants.BLUE,
                               -1)
                    cv2.putText(display_image,
                                f"Invalid: {invalid_waypoint_num}; Pos: {invalid_waypoint_pos}",
                                (int(invalid_waypoint_bb[0]), int(invalid_waypoint_bb[3]) + 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                constants.CVConstants.BLUE,
                                3)

            # Printing target center on the image.
            cv2.circle(display_image,
                       (int(self.target_x),
                        int(self.target_y)),
                       5,
                       constants.CVConstants.LIGHT_GREEN,
                       -1)
            # Plot the ideal trajectory of travel
            if valid_detection_point != [0, 0]:
                cv2.line(display_image,
                         (int(self.target_x), int(self.target_y)),
                         (valid_detection_point[0], valid_detection_point[1]),
                         constants.CVConstants.LIGHT_GREEN,
                         2,
                         cv2.LINE_8)

            display_image = self.bridge.cv2_to_imgmsg(np.array(display_image), "bgr8")
            # Publish to display topic (Can be viewed on localhost:8080).
            self.display_image_publisher.publish(display_image)
        except Exception as ex:
            self.get_logger().error(f"Failed publish qr detection visualization step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def get_valid_detection_delta(self, waypoints):
        """Method to get valid detection delta to be published for navigation
        from the detected waypoint information.

        Args:
            waypoints (Dictionary): Consisting of the waypoints decoded from QR Code

        Returns:
            valid_detection_delta (DetectionDeltaMsg): The delta_x, delta_y values to be published
            valid_detection_point (list): List of valid detection points.
            valid_waypoints (list): List of valid waypoints.
            invalid_waypoints (list): List of invalid waypoints.
        """
        try:
            invalid_waypoints = []
            valid_waypoints = []
            # Continue the ongoing action delta (None / any custom action delta)
            valid_detection_delta = self.ongoing_action
            valid_detection_point = [0, 0]
            # Keep track of last 3 waypoints detected
            # This is required in case more than 3 consecutive input images dont decode a QR Code,
            # we should publish a [0.0, 0.0] delta to trigger No Action in navigation.
            self.last_three_waypoints.append(len(waypoints))
            if len(self.last_three_waypoints) > 3:
                self.last_three_waypoints.pop(0)
            # Check if consecutive 3 input images did not detect a waypoint
            # and valid detection is set to None
            if not any(self.last_three_waypoints) and valid_detection_delta is None:
                # Publish a [0.0, 0.0] delta triggering a No Action in navigation.
                valid_detection_delta = self.calculate_delta(self.target_x,
                                                             self.target_y,
                                                             self.target_x,
                                                             self.target_y)
            else:
                # Find the average delta_x, delta_y of all the valid qr codes detected.
                delta_x = 0
                delta_y = 0
                for wp_num, wp in sorted(waypoints.items()):
                    detection_delta = self.calculate_delta(self.target_x,
                                                           self.target_y,
                                                           wp[constants.QRKeys.REFERENCE][0],
                                                           wp[constants.QRKeys.REFERENCE][1])
                    if abs(detection_delta.delta[1]) > constants.DELTA_DISTANCE_THRESHOLD_Y:
                        # Detection is valid only if the detected delta in y is greater than
                        # the threshold value. This helps to filter out detections which are too
                        # close to the sensor (already navigated across) and focus on the next QRs.
                        delta_x += detection_delta.delta[0]
                        delta_y += detection_delta.delta[1]
                        valid_detection_point[0] += wp[constants.QRKeys.REFERENCE][0]
                        valid_detection_point[1] += wp[constants.QRKeys.REFERENCE][1]
                        valid_waypoints.append(wp)
                    else:
                        invalid_waypoints.append(wp)
                if len(valid_waypoints) > 0:
                    # Calculate the delta to be published as an average of the delta values wrt
                    # the valid waypoints. This creates an average path for the car to follow when
                    # multiple QR codes influence the detection.
                    valid_detection_delta = DetectionDeltaMsg()
                    valid_detection_delta.delta = [delta_x / len(valid_waypoints),
                                                   delta_y / len(valid_waypoints)]
                    valid_detection_point = [valid_detection_point[0] // len(valid_waypoints),
                                             valid_detection_point[1] // len(valid_waypoints)]
            return valid_detection_delta, valid_detection_point, valid_waypoints, invalid_waypoints
        except Exception as ex:
            self.get_logger().error(f"Failed to get valid detection delta: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def run_waypoint_detection(self):
        """Method for running inference on received input image.
        """
        try:
            while not self.stop_thread:
                # Get an input image from double buffer.
                sensor_data = self.input_buffer.get()
                # Pre-process input.
                image = self.preprocess(sensor_data)
                # Decode the qr codes if present
                waypoints = self.decode_qr(image)
                # Calculate valid detection delta from acquired waypoints to be published
                valid_detection_delta, valid_detection_point, valid_waypoints, invalid_waypoints \
                    = self.get_valid_detection_delta(waypoints)
                # Publish the valid detection delta
                if valid_detection_delta is not None:
                    self.delta_publisher.publish(valid_detection_delta)
                # Add visualization on input image after QR Detection
                # and publish to DISPLAY_IMAGE_PUBLISHER_TOPIC
                self.publish_qr_detection_visualization(image,
                                                        valid_detection_point,
                                                        valid_waypoints,
                                                        invalid_waypoints)
        except Exception as ex:
            self.get_logger().error(f"Failed waypoint detection step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        qr_detection_node = QRDetectionNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number
                frame: the current stack frame (None or a frame object)
            """
            qr_detection_node.get_logger().info("Signal Handler initiated")
            qr_detection_node.thread_shutdown()
            qr_detection_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(qr_detection_node, executor)

    except Exception as ex:
        qr_detection_node.get_logger().error(f"Exception in QR Detection Node: {ex}")
        qr_detection_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
