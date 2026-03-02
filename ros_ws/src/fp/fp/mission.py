import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped
from gantry_interfaces.srv import ObjectPositions


class Mission(Node):
    def __init__(self):
        super().__init__("mission")

        self.declare_parameter("payload_topic", "/detection/payload")
        self.declare_parameter("dropping_zone_topic", "/detection/dropping_zone")
        self.declare_parameter("objects_service", "/object_positions")

        payload_topic = str(
            self.get_parameter("payload_topic").get_parameter_value().string_value
        )
        dropping_zone_topic = str(
            self.get_parameter("dropping_zone_topic").get_parameter_value().string_value
        )
        objects_service = str(
            self.get_parameter("objects_service").get_parameter_value().string_value
        )

        self.payload_subscriber = self.create_subscription(
            PointStamped,
            payload_topic,
            self.payload_callback,
            10,
        )
        self.dropping_zone_subscriber = self.create_subscription(
            PointStamped,
            dropping_zone_topic,
            self.dropping_zone_callback,
            10,
        )
        self.objects_client = self.create_client(ObjectPositions, objects_service)

        self.payload_position: tuple[float, float] | None = None
        self.dropping_zone_position: tuple[float, float] | None = None
        self.mission_started = False

        self.get_logger().info(
            f"Mission initialized | payload_topic: {payload_topic}"
            f" | dropping_zone_topic: {dropping_zone_topic}"
        )

    def payload_callback(self, msg: PointStamped):
        self.payload_position = (msg.point.x, msg.point.y)
        self.get_logger().info(f"Payload position updated: {self.payload_position}")
        if self.mission_ready():
            self.start_mission()

    def dropping_zone_callback(self, msg: PointStamped):
        self.dropping_zone_position = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f"Dropping zone position updated: {self.dropping_zone_position}"
        )
        if self.mission_ready():
            self.start_mission()

    def start_mission(self):
        if self.payload_position is None or self.dropping_zone_position is None:
            self.get_logger().warn(
                "Cannot start mission: payload or dropping zone position is missing"
            )
            return

        if self.mission_started:
            return
        self.mission_started = True

        self.get_logger().info("Mission started")
        self.objects_client.wait_for_service()

        request = ObjectPositions.Request()
        request.payload = Point()
        request.payload.x = self.payload_position[0]
        request.payload.y = self.payload_position[1]
        request.dropping_zone = Point()
        request.dropping_zone.x = self.dropping_zone_position[0]
        request.dropping_zone.y = self.dropping_zone_position[1]

        self.objects_client.call_async(request)

    def mission_ready(self):
        return (
            self.payload_position is not None
            and self.dropping_zone_position is not None
        )

    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
