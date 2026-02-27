from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
from cv_bridge import CvBridge


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("aruco_id", 39)
        self.aruco_id = int(self.get_parameter("aruco_id").value)

        # ROS setup
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.center_pub = self.create_publisher(PointStamped, "/aruco/pixel_center", 10)

        self.br = CvBridge()

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None:
            return

        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            if marker_id != self.aruco_id:
                continue

            marker_corners = corners[i].reshape(4, 2)

            # Compute pixel center (mean of 4 corners)
            center = marker_corners.mean(axis=0)
            u, v = float(center[0]), float(center[1])

            center_msg = PointStamped()
            center_msg.header.stamp = msg.header.stamp
            center_msg.header.frame_id = msg.header.frame_id or "camera_link"

            center_msg.point.x = u
            center_msg.point.y = v
            center_msg.point.z = 0.0  # pixel plane

            self.center_pub.publish(center_msg)

            # Optional visualization
            cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
            break  # publish only first matching marker

        cv2.imshow("Aruco Center", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()