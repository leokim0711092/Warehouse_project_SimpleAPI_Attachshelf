import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.substitutions import PathJoinSubstitution
from launch.actions import LogInfo


def generate_launch_description():
    


    map_file_arg = launch.substitutions.LaunchConfiguration('map_file')

    map_file_path = PathJoinSubstitution(
    [get_package_share_directory('map_server'), 'config', map_file_arg]
    )

    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map.rviz')


    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        rviz,
         LogInfo(msg=f"RViz config file path: {rviz_config_dir}"),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=["-map_file", map_file_arg],
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path} 
                       ]
            ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])            
        ])