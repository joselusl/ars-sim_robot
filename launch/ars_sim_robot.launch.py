from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'ars_sim_robot_dynamics_node_name', default_value='ars_sim_robot_dynamics_node',
            description='Name of the robot dynamics node'
        ),
        DeclareLaunchArgument(
            'ars_sim_robot_status_node_name', default_value='ars_sim_robot_status_node',
            description='Name of the robot status node'
        ),
        DeclareLaunchArgument(
            'screen', default_value='screen',
            description='Output setting for the nodes'
        ),
        
        # Launch the robot dynamics node
        Node(
            package='ars_sim_robot',
            executable='ars_sim_robot_dynamics_ros_node',
            name=LaunchConfiguration('ars_sim_robot_dynamics_node_name'),
            output=LaunchConfiguration('screen')
        ),
        
        # Launch the robot status node
        Node(
            package='ars_sim_robot',
            executable='ars_sim_robot_status_ros_node',
            name=LaunchConfiguration('ars_sim_robot_status_node_name'),
            output=LaunchConfiguration('screen')
        ),
    ])
