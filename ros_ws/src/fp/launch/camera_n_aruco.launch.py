from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package="camera",  # <-- Replace with your package name
        executable="camera_node",  # <-- Replace with your setup.py entry point
        name="usb_webcam",
        output="screen",
        parameters=[
            {
                "video_device": 2,  # Example param for camera index
                "framerate": 30.0,
            }
        ],
    )

    aruco_node = Node(
        package="aruco_detector",  # <-- Replace with your package name
        executable="aruco_detector",  # <-- Replace with your setup.py entry point
        name="aruco_processor",
        output="screen",
        parameters=[
            {
                "image_topic": "/camera/iamge_raw"  # Tell detector where to listen
            }
        ],
    )

    # 4. Return the Launch Description
    return LaunchDescription([camera_node, aruco_node])
