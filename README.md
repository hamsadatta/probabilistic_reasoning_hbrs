Overview
===============

Implementation of occupancy grid mapping and SLAM in simulation. There exists many assumptions in the current implementations. The main goal is to parameterise these assumptions and to build a generalised software.

### Using `gmapping_node`
-----------------------

#### Software versions:

    Ubuntu : 20.04  
    ROS2   : rolling  

- Install ROS2 and source the necessary files as described in the [ROS2 documentation](https://docs.ros.org/en/rolling/Installation.html). Clone the `probabilistic_reasoning_hbrs` repository.

- Install the dependencies of ROS2-rolling: `rosdep install -y -r -q --from-paths src --ignore-src --rosdistro rolling`

- build nav2_common, nav2_msgs, nav2_util, nav2_lifecycle_manager, nav2_map_server

- Current ROS2 related files sourced in `~/.bashrc` are as follows,

    ```  
    source /opt/ros/rolling/setup.bash  
    source /usr/share/colcon_cd/function/colcon_cd.sh  
    export _colcon_cd_root=/opt/ros/rolling/  
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash  

    stat /usr/share/gazebo/setup.sh &> /dev/null  
    if [ $? -eq 0 ]; then  
        source /usr/share/gazebo/setup.sh  
    fi    
    ```

Steps:

1. From the path, `probabilistic_reasoning_hbrs/src/occupancy_grid_mapping`, build the ROS2 package,
    > `colcon build --packages-select occupancy_grid_mapping`

2. Run the `local_setup.bash` file from the same path and in the same terminal,
    > `. install/local_setup.bash`

3. Considering `turtlebot3_house` environment, set the turtlebot3_model variable,
    > `export TURTLEBOT3_MODEL=waffle`

4. Launch the `turtlebot3_house` gazebo environment,
    > `ros2 launch  turtlebot3_gazebo turtlebot3_house.launch.py`

    > Note:  
        - While running this node for the first time it takes few minutes to load the environment.   
        - If the robot does not appear in simulation, please try launching the node again until the robot is loaded and the laser scans are visible. 
        

5. Run the `teleop_twist_keyboard` node to control turtlebot3 in simulation,
    > `ros2 run teleop_twist_keyboard teleop_twist_keyboard`


6. Run `gmapping_node` in the same terminal. A window showing the acquired map will be visible.
    > `ros2 run occupancy_grid_mapping gmapping_node`

7. Move around the turtlebot until required regions of the environment are perceived. Kill the terminal where the `gmapping_node` is running by pressing `Ctrl+C` and close the window showing the map to complete the mapping process.

8. The map will be saved as `.png` image at `probabilistic_reasoning_hbrs/src/occupancy_grid_mapping/install/occupancy_grid_mapping/share/occupancy_grid_mapping/`.
