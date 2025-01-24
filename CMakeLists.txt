cmake_minimum_required(VERSION 3.8)
project(franka_fer1_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(franka_moveit_config REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_plugins REQUIRED)
find_package(moveit_planners REQUIRED)
find_package(moveit_planners_chomp REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_resources REQUIRED)
find_package(moveit_runtime REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_task_constructor_capabilities REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_demo REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_task_constructor_visualization REQUIRED)
find_package(moveit_resources_panda_moveit_config REQUIRED)


add_executable(test_move_group src/test_move_group.cpp)
target_include_directories(test_move_group PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_move_group PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_move_group
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "moveit_visual_tools"
  "moveit_plugins"
  "moveit_planners"
  "moveit_planners_chomp"
  "moveit_planners_ompl"
  "moveit_common"
  "moveit_core"
  "moveit_resources"
  "moveit_runtime"
  "moveit_servo"
  "moveit_task_constructor_capabilities"
  "moveit_task_constructor_core"
  "moveit_task_constructor_demo"
  "moveit_task_constructor_msgs"
  "moveit_task_constructor_visualization"
  "moveit_resources_panda_moveit_config"
)

add_executable(test_motion_planning_pipeline src/test_motion_planning_pipeline.cpp)
target_include_directories(test_motion_planning_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_motion_planning_pipeline PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_motion_planning_pipeline
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "moveit_visual_tools"
  "moveit_plugins"
  "moveit_planners"
  "moveit_planners_chomp"
  "moveit_planners_ompl"
  "moveit_common"
  "moveit_core"
  "moveit_resources"
  "moveit_runtime"
  "moveit_servo"
  "moveit_task_constructor_capabilities"
  "moveit_task_constructor_core"
  "moveit_task_constructor_demo"
  "moveit_task_constructor_msgs"
  "moveit_task_constructor_visualization"
  "moveit_resources_panda_moveit_config"
)

install(TARGETS 
        test_move_group
        test_motion_planning_pipeline
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
