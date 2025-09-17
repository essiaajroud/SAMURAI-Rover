from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Load basic simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            'src', 'samurai_simulation', 'launch', 'simulation.launch.py')])
    )
    
    # ROS2 bridge for web interface
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket'
    )
    
    # Detection node
    detection_node = Node(
        package='samurai_detection',
        executable='yolo_node',
        name='yolo_detector'
    )
    
    # Tracking node
    tracking_node = Node(
        package='samurai_tracking',
        executable='tracking_node',
        name='target_tracker'
    )

    return LaunchDescription([
        simulation_launch,
        rosbridge_node,
        detection_node,
        tracking_node
    ])
