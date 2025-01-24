from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # DeclareLaunchArgument(
    #     "planning_plugin",
    #     default_value="ompl_interface/OMPLPlanner",
    #     description="Planner to use.",
    # )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="test_motion_planning_pipeline",
        package="franka_fer1_ros2",
        executable="test_motion_planning_pipeline",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            move_group_demo,   
        ]
    )
