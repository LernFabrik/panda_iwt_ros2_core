import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml



def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='172.16.0.2',
        description='Hostname or IP address of the robot.')

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')

    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))

    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                        'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
            ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                                'srdf',
                                                'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )

    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    planning_pipelines_config = PathJoinSubstitution(
        [
            FindPackageShare('franka_moveit_config'),
            "config",
            "planning_pipelines_config.yaml",
        ]
    )

    pilz_planning_pipeline_config = {
        "move_group": {},
        "robot_description_planning":{},
    }

    pilz_planning_yaml = load_yaml("franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml")
    pilz_planning_pipeline_config["move_group"].update(pilz_planning_yaml)

    pilz_cartesian_limits_yaml = load_yaml("franka_moveit_config", "config/pilz_cartesian_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_cartesian_limits_yaml)

    pilz_joint_limits_yaml = load_yaml("franka_moveit_config", "config/joint_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_joint_limits_yaml)

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
        pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    move_group_request_adapters = {
        "request_adapters": """default_planner_request_adapters/AddRuckigTrajectorySmoothing \
        default_planner_request_adapters/AddTimeOptimalParameterization"""
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    main_node = Node(
        # name="panda_main_node",
        package="panda_iwt_ros2_core",
        executable="panda_main",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            move_group_capabilities,
            move_group_request_adapters,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    return LaunchDescription(
        [
        robot_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,        
        main_node
        ]
    )
    