import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Node for video flow
    opencv_camera_node = ComposableNode(
        package='opencv_cam',
        plugin='opencv_cam::OpencvCamNode',
        name='opencv_camera',
        namespace='opencv_cam',
        parameters=[{
            'file': True,
            'filename': '/workspaces/isaac_ros-dev/videos/tv_at_3.mp4',
            'camera_info_path': '/workspaces/isaac_ros-dev/calibration_data/ost.yaml'
            #'fps': 30,
            #'height': 
        }],
        remappings=[('/image_raw', '/opencv_cam/image_raw')]
    )

    # Node for image process
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify',
        namespace='opencv_cam',
        remappings=[('image', 'image_raw'), ('camera_info', '/opencv_cam/camera_info')]
    )

    # Node for isaac ros apriltag
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        parameters=[{'size': 1.0,
                     'tag_family': 'tag36h11',
                     'max_tags': 4}],
        remappings=[('image', '/opencv_cam/image_raw'), ('camera_info', '/opencv_cam/camera_info')])

    # Container for composable nodes
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[opencv_camera_node, apriltag_node],
        output='screen'
    )

    return LaunchDescription([
        apriltag_container
    ])
