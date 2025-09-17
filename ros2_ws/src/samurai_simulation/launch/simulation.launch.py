from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_samurai_sim = get_package_share_directory('samurai_simulation')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    
    # Launch nodes
    detector_node = Node(
        package='samurai_detection',
        executable='yolo_node',
        name='yolo_detector',
        parameters=[{
            'model_path': 'server/models/best2.pt',
            'use_sim_time': True
        }]
    )
    
    # Simulated camera node
    camera_node = Node(
        package='gazebo_ros',
        executable='gazebo_camera',
        name='sim_camera',
        parameters=[{
            'frame_id': 'camera_link',
            'update_rate': 30.0
        }]
    )

    return LaunchDescription([
        gazebo,
        detector_node,
        camera_node
    ])
