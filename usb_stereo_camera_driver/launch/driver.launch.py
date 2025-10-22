from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    stereo_pipeline = ComposableNodeContainer(
        name='stereo_pipeline',
        package='rclcpp_components',
        namespace="",
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_stereo_camera_driver',
                plugin='stereoCamera::UsbStereoCameraDriver',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )
    return LaunchDescription([
        stereo_pipeline
    ])
