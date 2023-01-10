#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    map_server_config_path = '/home/melody/Desktop/MAS_course_materials/Courses/04Fourth\ Semester/PR/probabilistic_reasoning_hbrs/src/particle_filter/map/house_map.yaml'

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_server_config_path}])
        
    static_transform = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])


    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(static_transform)

    return ld
        
