from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class PixelToGroundNode(Node):
    def __init__(self) -> None:
        super().__init__("pixel_to_ground")

        self.declare_parameter("input_topic", "/aruco/pixel_center")
        self.declare_parameter("output_topic", "/aruco/ground_center")
        self.declare_parameter("homography_path", "homography_cm.npy")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        homography_path = Path(str(self.get_parameter("homography_path").value))

        self.homography = self.load_homography(homography_path)

        self.sub = self.create_subscription(
            PointStamped,
            input_topic,
            self.pose_callback,
            10,
        )
        self.pub = self.create_publisher(PointStamped, output_topic, 10)

        self.get_logger().info(
            f"PixelToGround node started | input: {input_topic} | output: {output_topic} | H: {homography_path}"
        )

    def load_homography(self, homography_path: Path) -> np.ndarray:
        if not homography_path.exists():
            raise FileNotFoundError(f"Homography file not found: {homography_path}")

        data = np.load(homography_path)

        if isinstance(data, np.ndarray):
            homography = data
        else:
            if "H" in data.files:
                homography = data["H"]
            elif "homography" in data.files:
                homography = data["homography"]
            else:
                raise KeyError("Homography .npz must contain key 'H' or 'homography'")

        homography = np.asarray(homography, dtype=np.float64)
        if homography.shape != (3, 3):
            raise ValueError(
                f"Homography must have shape (3, 3), got {homography.shape}"
            )

        return homography

    def pose_callback(self, msg: PointStamped) -> None:
        pixel_x = float(msg.point.x)
        pixel_y = float(msg.point.y)

        pixel = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        ground = cv2.perspectiveTransform(pixel, self.homography).reshape(2)

        out = PointStamped()
        out.header = msg.header
        out.point.x = float(ground[0])
        out.point.y = float(ground[1])
        out.point.z = float(msg.point.z)

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixelToGroundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
