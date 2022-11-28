Overview
===============

Implementation of occupancy grid mapping and SLAM in simulation. There exists many assumptions in the current implementations. The main goal is to parameterise these assumptions and to build a generalised software.

Description
====================

Occupancy Grip Mapping
------------------------

TODO: description of occupancy grid mapping

1. Considering neighborhood of a cell in the measurement model and parameterising the mapping task by its size.

2. Allowing different types of noise for sensor and odom data modeling.  

3. Based on confidence of localisation, updating the map. This might help to handle
dynamic obstacles

### Using `gmapping_node`
-----------------------

** Software versions: **

Ubuntu : 20.04  
ROS2   : rolling  

Install ROS2 and source the necessary files as described in the [ROS2 documentation](https://docs.ros.org/en/rolling/Installation.html). Clone the `probabilistic_reasoning_hbrs` repository.

Steps:

1. Considering `turtlebot3_house` environment, set the turtlebot3_model variable,
    > `export TURTLEBOT3_MODEL=waffle`

2. Launch the `turtlebot3_house` gazebo environment,
    > `ros2 launch  turtlebot3_gazebo turtlebot3_house.launch.py`

    > Note:   
        * While running this node for the first time it takes few minutes to load the environment.  
        * If the robot does not appear in simulation, please try launching the node couple of times until the robot is loaded. 
        

3. Run the `teleop_twist_keyboard` node to control turtlebot3 in simulation,
    > `ros2 run teleop_twist_keyboard teleop_twist_keyboard`


4. From the path "probabilistic_reasoning_hbrs/src/occupancy_grid_mapping", build the ROS2 package,
    > `colcon build --packages-select occupancy_grid_mapping`

5. Run the `local_setup.bash` file from the same path and in the same terminal,
    > `. install/local_setup.bash`

6. Run `gmapping_node` in the same terminal,
    > `ros2 run occupancy_grid_mapping gmapping_node`