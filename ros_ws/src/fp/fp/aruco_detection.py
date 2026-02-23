import math
from typing import Any, Dict, List, Optional, Tuple, cast

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray


ARUCO_DICT_MAP: Dict[str, int] = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def rvec_to_quaternion(rvec: np.ndarray) -> Tuple[float, float, float, float]:
    rot_mat, _ = cv2.Rodrigues(rvec)
    trace = float(rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2])

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot_mat[2, 1] - rot_mat[1, 2]) / s
        qy = (rot_mat[0, 2] - rot_mat[2, 0]) / s
        qz = (rot_mat[1, 0] - rot_mat[0, 1]) / s
    elif rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
        s = math.sqrt(1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2]) * 2.0
        qw = (rot_mat[2, 1] - rot_mat[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        qz = (rot_mat[0, 2] + rot_mat[2, 0]) / s
    elif rot_mat[1, 1] > rot_mat[2, 2]:
        s = math.sqrt(1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2]) * 2.0
        qw = (rot_mat[0, 2] - rot_mat[2, 0]) / s
        qx = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_mat[1, 2] + rot_mat[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1]) * 2.0
        qw = (rot_mat[1, 0] - rot_mat[0, 1]) / s
        qx = (rot_mat[0, 2] + rot_mat[2, 0]) / s
        qy = (rot_mat[1, 2] + rot_mat[2, 1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


class ArucoPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_pose_node")
        self.bridge = CvBridge()
        self.aruco_mod = cast(Any, cv2.aruco)

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("camera_matrix", [])
        self.declare_parameter("dist_coeffs", [])
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("annotated_image_topic", "/aruco/image_annotated")
        self.declare_parameter("pose_topic", "/aruco/poses")
        self.declare_parameter("ids_topic", "/aruco/ids")
        self.declare_parameter("draw_axes", True)

        self.image_topic = str(
            self._param_or_default("image_topic", "/camera/image_raw")
        )
        self.camera_info_topic = str(
            self._param_or_default("camera_info_topic", "/camera/camera_info")
        )
        self.camera_frame = str(self._param_or_default("camera_frame", "camera_link"))
        self.marker_length = float(self._param_or_default("marker_length", 0.05))
        self.publish_annotated_image = bool(
            self._param_or_default("publish_annotated_image", True)
        )
        self.annotated_image_topic = str(
            self._param_or_default("annotated_image_topic", "/aruco/image_annotated")
        )
        self.pose_topic = str(self._param_or_default("pose_topic", "/aruco/poses"))
        self.ids_topic = str(self._param_or_default("ids_topic", "/aruco/ids"))
        self.draw_axes = bool(self._param_or_default("draw_axes", True))

        dictionary_name = str(self._param_or_default("aruco_dictionary", "DICT_4X4_50"))
        dict_id = ARUCO_DICT_MAP.get(dictionary_name)
        if dict_id is None:
            self.get_logger().warn(
                f'Invalid aruco_dictionary "{dictionary_name}", fallback to DICT_4X4_50'
            )
            dict_id = self.aruco_mod.DICT_4X4_50

        self.aruco_dict = self.aruco_mod.getPredefinedDictionary(dict_id)
        self.detector_params = self.aruco_mod.DetectorParameters()
        self.aruco_detector = self.aruco_mod.ArucoDetector(
            self.aruco_dict, self.detector_params
        )

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._load_intrinsics_from_params()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )

        self.pose_pub = self.create_publisher(PoseArray, self.pose_topic, 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, self.ids_topic, 10)
        self.image_pub = None
        if self.publish_annotated_image:
            self.image_pub = self.create_publisher(
                Image, self.annotated_image_topic, 10
            )

        self._intrinsic_warned = False
        self.get_logger().info("Aruco pose node started")

    def _param_or_default(self, name: str, default: Any) -> Any:
        value = self.get_parameter(name).value
        if value is None:
            return default
        return value

    def _load_intrinsics_from_params(self) -> None:
        matrix_values = self._param_or_default("camera_matrix", [])
        dist_values = self._param_or_default("dist_coeffs", [])

        if isinstance(matrix_values, list) and len(matrix_values) == 9:
            self.camera_matrix = np.array(matrix_values, dtype=np.float64).reshape(3, 3)
        if isinstance(dist_values, list) and len(dist_values) >= 4:
            self.dist_coeffs = np.array(dist_values, dtype=np.float64).reshape(1, -1)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids = self._detect_markers(frame)

        ids_msg = Int32MultiArray()
        ids_msg.data = []
        pose_array = PoseArray()
        pose_array.header = msg.header
        if not pose_array.header.frame_id:
            pose_array.header.frame_id = self.camera_frame

        if ids is not None and len(ids) > 0:
            self.aruco_mod.drawDetectedMarkers(frame, corners, ids)
            marker_ids = [int(mid) for mid in ids.flatten()]
            ids_msg.data = marker_ids
            poses: List[Pose] = []

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                estimate_pose = getattr(self.aruco_mod, "estimatePoseSingleMarkers")
                rvecs, tvecs, _ = estimate_pose(
                    corners,
                    self.marker_length,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                for i in range(len(marker_ids)):
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    qx, qy, qz, qw = rvec_to_quaternion(rvec)

                    pose = Pose()
                    pose.position.x = float(tvec[0])
                    pose.position.y = float(tvec[1])
                    pose.position.z = float(tvec[2])
                    pose.orientation.x = qx
                    pose.orientation.y = qy
                    pose.orientation.z = qz
                    pose.orientation.w = qw
                    poses.append(pose)

                    if self.draw_axes:
                        cv2.drawFrameAxes(
                            frame,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            tvec,
                            self.marker_length * 0.5,
                        )
                pose_array.poses = poses
            elif not self._intrinsic_warned:
                self.get_logger().warn(
                    "No camera intrinsics yet. Pose estimation disabled until camera_info or "
                    "camera_matrix/dist_coeffs parameters are provided."
                )
                self._intrinsic_warned = True

        self.ids_pub.publish(ids_msg)
        self.pose_pub.publish(pose_array)

        if self.image_pub is not None:
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated.header = msg.header
            self.image_pub.publish(annotated)

    def _detect_markers(
        self, frame: np.ndarray
    ) -> Tuple[List[Any], Optional[np.ndarray]]:
        if hasattr(self, "aruco_detector"):
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            ids_np = None if ids is None else np.asarray(ids)
            return list(corners), ids_np

        corners, ids, _ = self.aruco_mod.detectMarkers(
            frame, self.aruco_dict, parameters=self.detector_params
        )
        ids_np = None if ids is None else np.asarray(ids)
        return list(corners), ids_np


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
