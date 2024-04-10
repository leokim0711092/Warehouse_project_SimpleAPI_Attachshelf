# Warehouse AMR - Move shelf to ship position

The Warehouse AMR Navigation System project is centered on developing an efficient, autonomous navigation system for the RB1 mobile robot. Our goal is to enable precise and adaptive movement within a dynamic warehouse environment.

## Workflow Overview:

1. Configuration of the Navigation System: 
- Set up and configuration of the RB1 robot's navigation system to ensure seamless initial operation within a controlled environment.

2. Integration with Simple Commander API:
- Utilization of the Simple Commander API to interact with the Nav2 system, creating navigation routes: 

4. Script Execution - move_shelf_to_ship.py: 
- Deployment of a script enabling the robot to localize itself within the warehouse and move to the designated loading position.

5. Shelf Handling Capabilities: 
- Under-shelf Navigation: Engineering the robot to navigate beneath the shelf, attach itself, and adapt its configuration for optimal maneuverability.
  
- Keepout Mask Activation: Activation of a visual keepout system using cones to mark restricted areas during the navigation system's startup.
  
- Shelf Transportation: Guiding the robot to the shipping position with the loaded shelf.

5. Return: 
- Unloading: Automated process for the robot to unload the shelf at the shipping dock.
  
- Repositioning: Ensuring the robot exits from beneath the shelf and returns to the initial starting position, ready for the next task.

## Requirements

- ROS2 (Robot Operating System 2)

- C++ Compiler

- Gazebo Simulation / Appropriate Robot Hardware
  
- Nav2

## Installation

1\. Clone the repository:

   ```
   git clone https://github.com/leokim0711092/Warehouse_project_SimpleAPI_Attachshelf.git
   ```

2\. Build the project:
   ```
   cd ~/ros2_ws
   colcon build && source install/setup.bash
   ```

## Simulation Environment
1\. Gazebo launch:
   ```
   source ~/simulation_ws/devel/setup.bash
   roslaunch rb1_base_gazebo warehouse_rb1.launch
   ```
2\. Rosbridge launch:
   ```
   source ~/simulation_ws/devel/setup.bash
   roslaunch load_params load_params_base.launch
   source /opt/ros/galactic/setup.bash
   ros2 run ros1_bridge parameter_bridge
   ```
## Usage
1\. Localization:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 launch localization_server localization.launch.py
   ```
2\. Path planner
   ```
   ros2 launch path_planner_server pathplanner.launch.py
   ```
3\. Move shelt to ship
   ```
   python3 ~/ros2_ws/src/nav2_apps/scripts/move_shelf_to_ship.py
   ```

## Demo

[Warehouse AMR demo
](https://www.youtube.com/watch?v=eYSYurGJ0aA&t=10s)

## Authors

- [@leokim0711092](https://github.com/leokim0711092)

## License
This project was created under the supervision of the team at [The Construct](https://theconstructsim.com/)
