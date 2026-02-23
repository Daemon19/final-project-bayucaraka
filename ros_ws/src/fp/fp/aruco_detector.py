from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("marker_size", 5.0)  # In centimeters
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.declare_parameter("aruco_id", 39)  # Default to marker ID 39
        self.aruco_id = int(self.get_parameter("aruco_id").value)

        # 2. Camera Calibration (REPLACE THESE WITH YOUR ACTUAL CALIBRATION)
        # You can get these from a 'camera_info' topic or a calibration file
        self.declare_parameter("calibration_path", "camera-calibration.npz")
        calibration_path = Path(self.get_parameter("calibration_path").value)
        self.camera_matrix, self.dist_coeffs = self.load_calibration(calibration_path)

        # 3. ROS Setup
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco/pose", 10)
        self.br = CvBridge()

        # 4. ArUco Setup (Humble/OpenCV 4.7+ style)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def load_calibration(self, calibration_path: Path) -> tuple[np.ndarray, np.ndarray]:
        if not calibration_path.exists():
            raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

        data = np.load(calibration_path)
        required_keys = {"camera_matrix", "distortion"}
        if not required_keys.issubset(data.files):
            raise KeyError(
                "Calibration file must contain keys: camera_matrix and distortion"
            )

        camera_matrix = data["camera_matrix"].astype(np.float64)
        distortion = data["distortion"].astype(np.float64)
        return camera_matrix, distortion

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None:
            self.get_logger().info("No markers detected.")
        else:
            rvecs, tvecs = self.estimate_pose(corners)

            for i in range(len(ids)):
                rvec = rvecs[i].reshape(3, 1)
                tvec = tvecs[i].reshape(3, 1)

                pose_msg = self.build_pose_msg(
                    msg.header.stamp, msg.header.frame_id, rvec, tvec
                )

                self.pose_pub.publish(pose_msg)

                # Draw the axis for visual debugging
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5,
                )

        cv2.imshow("Pose Estimation", frame)
        cv2.waitKey(1)

    def build_pose_msg(
        self, stamp, frame_id, rvec: np.ndarray, tvec: np.ndarray
    ) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id or "camera_link"

        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        rot_mat, _ = cv2.Rodrigues(rvec)
        trace = rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rot_mat[2, 1] - rot_mat[1, 2]) * s
            qy = (rot_mat[0, 2] - rot_mat[2, 0]) * s
            qz = (rot_mat[1, 0] - rot_mat[0, 1]) * s
        elif rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2])
            qw = (rot_mat[2, 1] - rot_mat[1, 2]) / s
            qx = 0.25 * s
            qy = (rot_mat[0, 1] + rot_mat[1, 0]) / s
            qz = (rot_mat[0, 2] + rot_mat[2, 0]) / s
        elif rot_mat[1, 1] > rot_mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2])
            qw = (rot_mat[0, 2] - rot_mat[2, 0]) / s
            qx = (rot_mat[0, 1] + rot_mat[1, 0]) / s
            qy = 0.25 * s
            qz = (rot_mat[1, 2] + rot_mat[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1])
            qw = (rot_mat[1, 0] - rot_mat[0, 1]) / s
            qx = (rot_mat[0, 2] + rot_mat[2, 0]) / s
            qy = (rot_mat[1, 2] + rot_mat[2, 1]) / s
            qz = 0.25 * s

        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)

        return pose_msg

    def estimate_pose(self, corners):
        half = float(self.marker_size) / 2.0
        obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        rvecs = []
        tvecs = []
        for marker_corners in corners:
            img_points = np.asarray(marker_corners, dtype=np.float32).reshape(-1, 2)
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not success:
                rvec = np.zeros((3, 1), dtype=np.float32)
                tvec = np.zeros((3, 1), dtype=np.float32)
            rvecs.append(rvec)
            tvecs.append(tvec)

        return np.asarray(rvecs), np.asarray(tvecs)


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
