from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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

    camera_node = Node(
        package="fp",
        executable="camera",
        name="camera",
        output="screen",
        parameters=[
            {
                "camera_index": LaunchConfiguration("camera_index"),
                "camera_calibration": LaunchConfiguration("calibration_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_index_arg,
            calibration_path_arg,
            homography_path_arg,
            mission_node,
            camera_node,
        ]
    )
