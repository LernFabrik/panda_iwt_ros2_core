# Panda IWT ROS2
This is the core package for controlling the Franka Emika Panda robot. This package, along with the [PLC Controller](https://github.com/LernFabrik/panda_iwt_ros2_plc_controller) and the [Interfaces definition](https://github.com/LernFabrik/panda_iwt_ros2_interfaces), constitute the necessary package to control the robot with the conveyor belt system. These packages serve the purpose of migrating the usage from the previously installed KUKA robot, and are an adaptation of its control pa

## Usage
1. To start the system:
  - Make sure the main PLC is running
  - Make sure the USB connection from the ROS PC to the PLC has the IP `192.168.0.222` and that the ethernet connection to the robot has the IP `172.16.0.1` (The panda robot has the IP `172.16.0.2`)
2. Release the breaks of the robot
3. Activate FCI from the top right menu
4. Make sure the execution button of the robot (black) is released in order to command the robot in automatic mode (blue LEDs on the robot)
5. In 3 different terminals, execute the following commands (in sequence):

    Start the connection with the robot using:
    ```
    docker start panda_container_3
    docker exec -it panda_container_3 /bin/bash
    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 load_gripper:=true use_fake_hardware:=false
    ```
    Then wait for rViz to successfully start. If the start is unsuccessful (for example, due to a crash), then press the execution button of the robot (black) and release it, and also restart the FCI from the tablet.

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
    Note: a persistent issue exis that cause the connection to the PLC to randomly timeout. In such cases, please restart the node. The system should continue from the last state.A possible mitigation (future work) would be to wrap the PLC controller node in a launch file and a respawn argument to restart the node upon shutdown

6. Start the "Fertigungskonzept" program from the PLC HMI screen

## Notes
- **Important**: Please do *not* upgrade the packages within the docker container since packages used are re-adapted for the Franka Emika Panda robot for which the official support is dropper. Thus, upgrading the docker packages results in pacakges such as ros-humble-libraka to be installed (which are developed for Franka Research 3 robot) and thus causing conflicting behavior. However, if the upgrade takes place, then please reinstall the docker container as explained below
- **Important**: Only 'fatal' logging messages are displayed from the controller_manger. This is to avoid the [recent update](https://github.com/ros-controls/ros2_control/pull/1969) of ros2_controller which continuously published messages on the terminal for failed updates of the robot state. Please be aware of this behavior and dsiable it by removing the arguments field of the ros_controller node when launching it from the franka_moveit_config moveit.launch.py file
- Initial homing of the gripper was disabled due to a recent bug (28.02.2025) and replaced with initially openning the gripper. Further checks are needed
- The table where the placement is being added should be free when the program starts since the robot would initially not be aware of the occupied slots
- The conveyor system does not start the pneumatic pump (for the brakes) when the system is started, until the robot is homed. Please keep the area from the EOL to the robot free of object initially
- Dock the EOL cell into the conveyor system to start its functionality



## Reinstallation
Follow the following steps for reinstalling the container:
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


8. Clone panda_iwt_* pacakges (i.e: [](), [](), []() ) into the franka_ros_ws/src directory

9. Build everthing 
    ```
    colcon build --event-handlers desktop_notification+ status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
10. In the /franka_ros_ws directory (not the src) execute `source install/setup.bash` to update the shell with the built changes

11. Execute the follwoing within the container 
    ```
    sudo apt install iproute2 htop nano iputils-ping git-core bash-completion python3-venv python3-pip`
    pip install python-snap7==1.2
    ```
     
12. Proceed with the usage steps mentioned above


## Further Notes (IWT)
- The source code for the PLC program is on the NAS (N://Geräte, Maschinen Lernfabrik/Bandumlaufsystem)
- Connection schematic could be found on the Sharepoint (Dokumente -> 03 Produktion -> 10_LernFabrik -> 00_Adminstration -> Bandumlauf System Network -> bandumlauf_netwrok_latest.png). Note: that the schemtic considers the KUKA robot, the IPs, of which are simply replaced by those menthioned above


### Contributors
- Hazem Youssef