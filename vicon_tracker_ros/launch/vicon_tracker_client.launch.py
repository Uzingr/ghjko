import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node_vicon_client = Node(
        package='vicon_tracker_ros',
        executable='vicon_tracker_client',
        name='vicon_tracker_client',
        parameters=[os.path.join(
            get_package_share_directory('vicon_tracker_ros'),
            'config',
            'vicon_tracker_client_params.yaml'
        )]
    )

    node_pose_to_odometry = Node(
        package='vicon_tracker_ros',
        executable='pose_to_odometry',
        name='pose_to_odometry'
    )

    ld.add_action(node_vicon_client)
    ld.add_action(node_pose_to_odometry)
    return ld
