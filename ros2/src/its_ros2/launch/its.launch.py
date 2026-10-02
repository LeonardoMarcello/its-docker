import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Declare Launch Arguments
    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='standard',
        description='Choose which ITS algorithm to run: "standard" or "soft"'
    )

    setup_file_arg = DeclareLaunchArgument(
        'setup',
        default_value='setup.yaml',
        description='Choose which Setup configuration uses'
    )

    # namespace_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='ft',
    #     description='Namespace for the nodes'
    # )

    # 2. Find the setup.yaml configuration file
    # Make sure 'its_ros2' matches your exact package name!
    config_file = PathJoinSubstitution([
        FindPackageShare('its_ros2'),
        'config',
        LaunchConfiguration('setup')
    ])

    # 3. Define the Nodes
    # Contact Sensing Problem Solver
    its_node = Node(
        package='its_ros2',
        executable='its_node',
        name='its_node',
        #namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'standard'"])
        )
    )
    soft_its_node = Node(
        package='its_ros2',
        executable='soft_its_node',
        name='soft_its_node',
        #namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'soft'"])
        )
    )

    # Contact Sensing Problem Visuaizer
    soft_its_viz = Node(
        package='its_ros2',
        executable='soft_its_viz',
        name='soft_its_viz',
        #namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[config_file]
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        algorithm_arg,
        #namespace_arg,
        its_node,
        soft_its_node,
        soft_its_viz
    ])
