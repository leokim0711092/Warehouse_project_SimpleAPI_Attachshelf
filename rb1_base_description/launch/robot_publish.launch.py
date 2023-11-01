import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro
    


def generate_launch_description():

    ####### DATA INPUT ##########
    # robot_desc_path = os.path.join(get_package_share_directory('rb1_base_description'), "robots", 'rb1_base.urdf')

    # Robot State Publisher

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
    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher
        ]
    )