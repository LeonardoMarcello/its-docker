import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PythonExpression,
    PathJoinSubstitution, Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch Arguments                                                    #
    # ------------------------------------------------------------------ #
    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='standard',
        description='ITS algorithm: "standard" or "soft"'
    )

    finger_frame_arg = DeclareLaunchArgument(
        'finger_frame',
        default_value='fingertip',
        description='TF frame id of the fingertip link in the URDF'
    )

    mesh_file_arg = DeclareLaunchArgument(
        'mesh_file',
        default_value='package://its_ros2/assets/meshes/fingertip.stl',
        description='URI of the fingertip mesh (package:// or absolute path)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # ------------------------------------------------------------------ #
    #  Paths                                                               #
    # ------------------------------------------------------------------ #
    pkg_share  = FindPackageShare('its_ros2')

    config_file = PathJoinSubstitution([pkg_share, 'config', 'setup.yaml'])
    urdf_file = PathJoinSubstitution([pkg_share, 'assets', 'urdf', 'fingertip.urdf.xacro'])
    rviz_file   = PathJoinSubstitution([pkg_share, 'config', 'rviz',   'its.rviz'])

    # ------------------------------------------------------------------ #
    #  Robot description (xacro → URDF string)                            #
    # ------------------------------------------------------------------ #
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' finger_frame:=', LaunchConfiguration('finger_frame'),
            ' mesh_file:=',    LaunchConfiguration('mesh_file'),
        ]),
        value_type=str
    )

    # ------------------------------------------------------------------ #
    #  Nodes                                                               #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    its_node = Node(
        package='its_ros2',
        executable='its_node',
        name='its_node',
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
        output='screen',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'soft'"])
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ------------------------------------------------------------------ #
    #  Launch Description                                                  #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        algorithm_arg,
        finger_frame_arg,
        mesh_file_arg,
        rviz_arg,
        robot_state_publisher,
        its_node,
        soft_its_node,
        rviz_node,
    ])
