# Panda IWT ROS2
This is the core package for controlling the Franka Emika Panda robot. This package, along with the [PLC Controller](https://github.com/LernFabrik/panda_iwt_ros2_plc_controller) and the [Interfaces](https://github.com/LernFabrik/panda_iwt_ros2_interfaces), constitute the necessary packages to control the robot with the conveyor belt system. These packages serve the purpose of migrating the usage from the previously installed KUKA robot and build on top of its control scripts [Link](https://github.com/LernFabrik/kuka_iiwa7_ros2).

## Usage
1. To start the system:
  - Make sure the main PLC is running
  - Make sure the USB connection from the ROS PC to the PLC has the IP `192.168.0.222` and that the ethernet connection to the robot has the IP `172.16.0.1` (The panda robot has the IP `172.16.0.2`)
2. Release the brakes of the robot from the panda app on the tablet
3. Activate FCI from the top right menu
4. Make sure the execution button of the robot (black) is released in order to command the robot in automatic mode (blue LEDs on the robot)
5. In 3 different terminals, execute the following commands (in sequence):

    Start the connection with the robot using:
    ```
    xhost +local:root
    docker start panda_container_3
    docker exec -it panda_container_3 /bin/bash
    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true use_fake_hardware:=false
    ```
    Then wait for rViz to successfully start. 
    
    **Note**: If the start is unsuccessful (for example, due to a robot crash), then press the execution button of the robot (black) and release it, and also restart the FCI from the tablet.

    Start the robot control pipeline:
    ```
    docker exec -it panda_container_3 /bin/bash
    ros2 launch panda_iwt_ros2_core panda_main.launch.py
    ```
    Then you should see the message: `Doing nothing...`, indicating that the system is ready.

    Finally, start the PLC controller:
    ```
    docker exec -it panda_container_3 /bin/bash
    ros2 run panda_iwt_ros2_plc_controller plc_controller_node 
    ```
    The last message message: `Use Table: True` indicates that the connection to the PLC is ready.
    
    **Note**: a persistent issue exis that cause the connection to the PLC to randomly timeout. In such cases, please restart the node. The system should continue from the last state.A possible mitigation (future work) would be to wrap the PLC controller node in a launch file and a respawn argument to restart the node upon shutdown

6. Start the "Fertigungskonzept" program from the PLC HMI screen

## Notes
- **Important**: Please do *not* upgrade the packages within the docker container since packages used are re-adapted for the Franka Emika Panda robot for which the official support is dropped. Thus, upgrading the docker packages results in pacakges such as ros-humble-libraka to be installed (which are developed for Franka Research 3 robot) and thus causing conflicting behavior. However, if the upgrade takes place, then please delete the container and reinstall it as explained below
- **Important**: Only 'fatal' logging messages are displayed from the controller_manger node. This is to avoid the [recent update](https://github.com/ros-controls/ros2_control/pull/1969) of ros2_controller which continuously published messages on the terminal for failed updates of the robot state. Please be aware of this behavior and disable it by removing the arguments field of the *ros2_control_node* node when launching it from the franka_moveit_config moveit.launch.py file
- Initial homing of the gripper was disabled due to a recent bug (28.02.2025) and replaced by initial openning the gripper. Further checks are needed
- **Important**: The table where the placement is being added should be free when the program ("Fertigungskonzept") starts since the robot would initially not be aware of the occupied slots
- **Important**: The conveyor system does not start the pneumatic pump (for the brakes) when the system is started, until the robot is homed. Please keep the area from the EOL to the robot free of objects initially
- Dock the EOL cell into the conveyor system to start its functionality
- To modify the scripts in the container, it is best to use VS Code and attach to the running container (through the docker extension) 


## Direct Robot Control
- The robot could be directly commanded independent from the PLC. For that, please execute the first 2 code blocks (until the PLC controller node) mentioned above in the **Usage** section. The robot can then be commanded to execute the different functionalities by:
```
ros2 topic pub --once panda_control panda_iwt_ros2_interfaces/msg/PandaControl "{move_home: true, conveyor_pick: false, hochregallager_pick: false, table_pick: false, slot_id: 0}"
```
The above fields could be activated (only one at a time) or modified to perform the desired maneuver.
 
Other commands for controlling the gripper can also be used to home the gripper or open/close it:
```
ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing "{}"
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 10.0}}"
```
Note that the position range of the gripper is between 0.0 and 0.04. The max_effort field is ignored.

- The robot can also be directly commanded via rViz by activating the *MotionPlanning* plugin and moving the robot, then pressing *plan & execute*. However, commanding the robot this way is limited to simple motion commands.


## Teaching new poses
1. Switch to manual mode (by pressing the execution button (black))
2. Move the robot to the desired pose
3. Switch to automatic mode (release the execution button)
4. Activate FCI from the top right window of the panda app on the tablet
5. Open a terminal and execute the following to connect to the robot:
    ```
    docker start panda_container_3
    docker exec -it panda_container_3 /bin/bash
    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true use_fake_hardware:=false
    ```
6. Get the current position readings of the robot:
    ```
    ros2 run tf2_ros tf2_echo world panda_hand
    ```
7. Copy the values to the respective location (for example: "hochregallager_pose", "table_pose_0_pick", "conveyor_pose", etc) in the *main.cpp* file of the *panda_iwt_ros2_core* package 

7. Build the wokspace using
    ```
    colcon build --event-handlers desktop_notification+ status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
8. Source the changes by executing `source install/setup.bash` in the /franka_ros_ws directory (not the src)

**Extra Note**: The *joint* angles of the robot could be obtained via `ros2 topic echo /franka/joint_states`. However, please note the order of the joint is: panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint6, panda_joint7, panda_joint5 and thus should be corrceted if used (i.e: when modifying the behavior of the pick-place pipeline itself).



## Reinstallation
- **Note**: A snapshot of the current docker container is created and could be listed by running `docker images` on a ROS PC terminal("panda_container_3:latest"). The image could then be used to create the container (refer to the first point below - the remaining steps should not be carried out)

- To reinstall the container from scrach, follow the following steps:
1. Create the container using:
    ```
    docker run -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --name="panda_container_3" \
        --privileged \
        --net=host \
        osrf/ros:humble-desktop-full-jammy \
        /bin/bash
    export containerId=$(docker ps -l -q)
    ```

2. Install [libfranka](https://github.com/frankaemika/libfranka) (tag 0.9.2) in the /root directory of the container

3. Install moveit from source (https://moveit.ai/install-moveit2/source/) 

4. Create a ros workspace `mkdir -p franka_ros_ws/src` in the /root directory

5. Clone the LernFabrik fork of [franka_ros2](https://github.com/LernFabrik/franka_ros2) in the franka_ros_ws/src directory

6. Install [moveit_visual_tools](https://github.com/moveit/moveit_visual_tools) from source [change distro from foxy to humble in the rosdep command]

7. (optional) Install [moveit_task_constructor](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) from source --> Note: Task constructor is not used in the current setup


8. Clone panda_iwt_* pacakges, i.e: [panda_iwt_ros2_core](https://github.com/LernFabrik/panda_iwt_ros2_core), [panda_iwt_ros2_plc_controller](https://github.com/LernFabrik/panda_iwt_ros2_plc_controller), and [panda_iwt_ros2_interfaces](https://github.com/LernFabrik/panda_iwt_ros2_interfaces) into the franka_ros_ws/src directory

9. Build everthing 
    ```
    colcon build --event-handlers desktop_notification+ status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
10. In the /franka_ros_ws directory (not the src) execute `source install/setup.bash` to update the shell with the built changes

11. Execute the following within the container 
    ```
    sudo apt install iproute2 htop nano iputils-ping git-core bash-completion python3-venv python3-pip
    pip install python-snap7==1.2
    ```

12. Add the following to the /root/.bashrc file and source it:

    ```
    source /opt/ros/humble/setup.bash
    source /root/franka_ros_ws/install/local_setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export RCUTILS_COLORIZED_OUTPUT=1 # Terminal colorization for RCLCPP loggers
    ```

13. Proceed with the usage steps mentioned above


## Further Notes (IWT)
- The source code for the PLC program is on the NAS (N://Geräte, Maschinen Lernfabrik/Bandumlaufsystem)
- Network connection schematic for the Bandumlaufsystem could be found on the Sharepoint (Dokumente -> 03 Produktion -> 10_LernFabrik -> 00_Adminstration -> Bandumlauf System Network -> bandumlauf_netwrok_latest.png). Note: that the schematic considers the KUKA robot, the IPs, of which are simply replaced by those menthioned above for the panda robot


### Contributors
- Hazem Youssef