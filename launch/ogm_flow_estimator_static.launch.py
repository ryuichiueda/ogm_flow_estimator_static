from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch the ogm_flow_estimator_static node and optionally RViz with a specific configuration.
    
    Launch Arguments:
        use_rviz: Whether to launch RViz (default: true)
    """
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Get the RViz configuration file path
    rviz_config_file = os.path.join(
        get_package_share_directory('flow_estimator'),
        'rviz', 'ogm_flow_estimator_static.rviz'
    )
    
    # Define launch actions
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set to "true" to launch RViz'
    )
    
    ogm_flow_estimator_node = Node(
        package='flow_estimator',
        executable='flow_estimator',
        name='flow_estimator',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(use_rviz)
    )
    
    # Compose launch description
    return LaunchDescription([
        declare_use_rviz,
        ogm_flow_estimator_node,
        rviz_node
    ])
