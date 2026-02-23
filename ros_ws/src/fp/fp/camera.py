import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera(Node):
    CAPTURE_PERIOD = 1 / 30

    def __init__(self):
        super().__init__("camera")

        self.declare_parameter("camera_index", 0)

        # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        # self.publisher_ = self.create_publisher(Image, "camera/image_raw", qos_profile)
        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        camera_index = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        self.capture = cv2.VideoCapture(camera_index)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # type: ignore
        self.bridge = CvBridge()
        self.capture_timer = self.create_timer(self.CAPTURE_PERIOD, self.capture_image)
        self.get_logger().info("Camera node has been started.")

    def capture_image(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Captured an image.")
        self.publish_image(image_msg)

    def publish_image(self, image_msg: Image):
        self.publisher_.publish(image_msg)
        self.get_logger().info("Published an image.")

    def destroy_node(self):
        self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
