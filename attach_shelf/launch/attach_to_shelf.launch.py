import launch
from launch_ros.actions import Node

# How to use Example:
# ros2 launch execution_and_callbacks_examples start_with_arguments.launch.py timer_period:=0.5


def generate_launch_description():


    obstacle_arg = launch.substitutions.LaunchConfiguration('obstacle', default= 0.0)
    degrees_arg = launch.substitutions.LaunchConfiguration('degrees', default= 0)
    final_approach_arg = launch.substitutions.LaunchConfiguration('final_approach', default= False)

    approach_server = Node(
        package='attach_shelf',
        executable='approach_test',
        output='screen',
        emulate_tty=True
        )


    pre_approach = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        emulate_tty=True,
        arguments=["-obstacle", obstacle_arg , "-degrees", degrees_arg,
            "-final_approach", final_approach_arg
        ] )



    return launch.LaunchDescription(
    [
        approach_server,
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('obstacle')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('degrees')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('final_approach')),
        pre_approach
    ]    
    )