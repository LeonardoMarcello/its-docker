import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('its_ros2')
    yaml_file = os.path.join(pkg_share, 'config', 'ahand_bringup.yaml')
    wrench_compensator_yaml_file = os.path.join(pkg_share, 'config', 'bias_correction.yaml')

    # 1. Load the custom YAML file
    with open(yaml_file, 'r') as f:
        config_data = yaml.safe_load(f)

    ld = LaunchDescription()

    # 2. Extract the shared/global parameter blocks
    shared_params = {}
    launch_viz = False
    launch_compensator = config_data.get("compensate_fingertips", False)
    if 'soft_its' in config_data:
        shared_params['soft_its'] = config_data['soft_its']
    if 'soft_viz' in config_data:
        shared_params['soft_viz'] = config_data['soft_viz']
        launch_viz = True

    # 3. Loop through the config to find and launch sensor nodes
    for key, value in config_data.items():
        # Skip the shared parameter blocks (we already extracted them)
        if key in ['soft_its', 'soft_viz', "compensate_fingertips"]:
            continue

        # Merge the sensor-specific params with the shared params
        node_params = value.copy()
        node_params.update(shared_params)

        # Launch a node for this specific sensor
        node = Node(
            package='its_ros2',
            executable='its_node',
            name=key,              # 'sensor_1', 'sensor_2', etc.
            #namespace=key,         # Puts topics under /sensor_1/...
            output='screen',
            parameters=[node_params] # Pass the merged Python dictionary
        )
        ld.add_action(node)



        if launch_viz:
            # Contact Sensing Problem Visuaizer
            node_viz = Node(
                package='its_ros2',
                executable='soft_its_viz',
                name='soft_its_viz',
                #namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[node_params]
            )
            ld.add_action(node_viz)

    # wrench compensation node
    if launch_compensator:
        comp_node = Node(
            package='its_ros2',
            executable='wrench_compensator',
            name='wrench_compensator',
            output='screen',
            parameters=[wrench_compensator_yaml_file]
        )
        ld.add_action(comp_node)

    return ld
