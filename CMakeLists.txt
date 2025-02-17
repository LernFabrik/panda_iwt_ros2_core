cmake_minimum_required(VERSION 3.8)
project(panda_iwt_ros2_core)

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
find_package(franka_msgs REQUIRED)
find_package(franka_gripper REQUIRED)
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
find_package(panda_iwt_ros2_interfaces REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/panda_manipulation.cpp
  src/create_motion_planning.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "franka_msgs"
  "franka_gripper"
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
  "moveit_resources_panda_moveit_config"
  "panda_iwt_ros2_interfaces"
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

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
  "moveit_resources_panda_moveit_config"
  # "moveit_task_constructor_capabilities"
  # "moveit_task_constructor_core"
  # "moveit_task_constructor_demo"
  # "moveit_task_constructor_msgs"
  # "moveit_task_constructor_visualization"
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
  "moveit_resources_panda_moveit_config"
  # "moveit_task_constructor_capabilities"
  # "moveit_task_constructor_core"
  # "moveit_task_constructor_demo"
  # "moveit_task_constructor_msgs"
  # "moveit_task_constructor_visualization"
)


add_executable(panda_main src/main.cpp)
target_link_libraries(panda_main ${PROJECT_NAME})
target_include_directories(panda_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_compile_features(panda_main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  panda_main
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "franka_msgs"
  "franka_gripper"
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
  "moveit_resources_panda_moveit_config"
  "panda_iwt_ros2_interfaces"
)

add_executable(create_motion_planning src/create_motion_planning.cpp)
target_link_libraries(create_motion_planning ${PROJECT_NAME})
target_include_directories(create_motion_planning PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_compile_features(create_motion_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  create_motion_planning
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "franka_msgs"
  "franka_gripper"
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
  "moveit_resources_panda_moveit_config"
  "panda_iwt_ros2_interfaces"
)

add_executable(panda_manipulation src/panda_manipulation.cpp)
target_link_libraries(panda_manipulation ${PROJECT_NAME})
target_include_directories(panda_manipulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_compile_features(panda_manipulation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  panda_manipulation
  "moveit"
  "moveit_ros_move_group"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_ros"
  "moveit_ros_robot_interaction"
  "moveit_ros_visualization"
  "franka_moveit_config"
  "franka_msgs"
  "franka_gripper"
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
  "moveit_resources_panda_moveit_config"
  "panda_iwt_ros2_interfaces"
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
        panda_main
        test_move_group
        test_motion_planning_pipeline
        create_motion_planning
        panda_manipulation
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

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



ament_package()
