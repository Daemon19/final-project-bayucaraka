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

    camera_node = Node(
        package="fp",
        executable="camera",
        name="camera",
        output="screen",
        parameters=[
            {
                "camera_index": LaunchConfiguration("camera_index"),
            }
        ],
    )

    aruco_node = Node(
        package="fp",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[
            {
                "calibration_path": LaunchConfiguration("calibration_path"),
            }
        ],
    )

    pixel_to_ground_node = Node(
        package="fp",
        executable="pixel_to_ground",
        name="pixel_to_ground",
        output="screen",
        parameters=[
            {
                "homography_path": LaunchConfiguration("homography_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_index_arg,
            calibration_path_arg,
            homography_path_arg,
            camera_node,
            aruco_node,
            pixel_to_ground_node,
        ]
    )
