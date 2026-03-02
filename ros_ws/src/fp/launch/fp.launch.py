from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    camera_index_arg = DeclareLaunchArgument("camera_index", default_value="0")
    calibration_path_arg = DeclareLaunchArgument(
        "calibration_path", default_value="camera_calibration.npz"
    )
    homography_path_arg = DeclareLaunchArgument(
        "homography_path", default_value="homography_cm.npy"
    )

    mission_node = Node(
        package="fp",
        executable="mission",
        name="mission",
        output="screen",
        parameters=[{}],
    )

    return LaunchDescription(
        [
            camera_index_arg,
            calibration_path_arg,
            homography_path_arg,
            mission_node,
        ]
    )
