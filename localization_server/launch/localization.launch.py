import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch.actions import LogInfo
import xacro

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    map_file_arg = launch.substitutions.LaunchConfiguration('map_file', default= 'warehouse_map_sim.yaml')

    map_file_path = PathJoinSubstitution(
    [get_package_share_directory('map_server'), 'config', map_file_arg]
    )

    # rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localization.rviz')


    robot_model_path = os.path.join(
        get_package_share_directory('rb1_base_description'))

    xacro_file = os.path.join(robot_model_path, 'robots', 'rb1_base.urdf')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )



    return LaunchDescription([

        robot_state_publisher,

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     name='rviz_node',
        #     parameters=[{'use_sim_time': False}],
        #     arguments=['-d', rviz_config_dir]),

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
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])