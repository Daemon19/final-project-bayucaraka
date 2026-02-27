from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index_arg = DeclareLaunchArgument("camera_index", default_value="0")
    calibration_path_arg = DeclareLaunchArgument(
        "calibration_path", default_value="camera-calibration.npz"
    )
    homography_path_arg = DeclareLaunchArgument(
        "homography_path", default_value="homography_cm.npy"
    )

    perception_node = Node(
        package="fp",
        executable="perception",
        name="perception",
        output="screen",
        parameters=[
            {
                "camera_index": LaunchConfiguration("camera_index"),
                "camera_calibration": LaunchConfiguration("calibration_path"),
                "homography_path": LaunchConfiguration("homography_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_index_arg,
            calibration_path_arg,
            homography_path_arg,
            perception_node,
        ]
    )
