#include "panda_manipulation.hpp"

#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>


using namespace std::placeholders;

namespace iwtros2
{
/**
 * @brief Construct a new Panda Move:: Panda Move object
 *
 * @param options ros2 options
 */
PandaMove::PandaMove(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
    : _node(node), _group(group)
{
    RCLCPP_INFO(_node->get_logger(), "Move group interface node is starting");
    // todo initiallize more variables
    // Initialize sub and pub
    // Initialize parameretes

    // Display basic information
    RCLCPP_INFO(_node->get_logger(), "Planning frame: %s", _group->getPlanningFrame().c_str());
    RCLCPP_INFO(_node->get_logger(), "End effector link: %s", _group->getEndEffectorLink().c_str());

    this->_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        _node, "panda_link0", "trajectory_marker", _group->getRobotModel());
    _visual_tools->deleteAllMarkers();
    //_visual_tools->loadRemoteControl();

    this->_text_pose = Eigen::Isometry3d::Identity();
    _text_pose.translation().z() = 1.0;
    _visual_tools->publishText(_text_pose, "MoveGroupInterface_Panda", rvt::WHITE, rvt::XLARGE);
    _visual_tools->trigger();

    // Initialize Gripper
    this->_gripper_motion_client_ptr_ = rclcpp_action::create_client<GripperCommandAction>(_node,"/panda_gripper/gripper_action"); 

    // Gripper homing
    this->_gripper_homing_client_ptr_ = rclcpp_action::create_client<GripperHomingAction>(_node,"/panda_gripper/homing");
    // this->home_gripper();

    // Create motion planning
    this->_motion = std::make_shared<CreateMotion>(_node, _group);
}

geometry_msgs::msg::PoseStamped PandaMove::generatePose(const double x, const double y, const double z,
                                                       const double roll, const double pitch, const double yaw,
                                                       const std::string baselink)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = baselink.c_str();
    pose.header.stamp = rclcpp::Clock().now();

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
}

void PandaMove::home_gripper()
{
    // using namespace std::placeholders;

    if (!this->_gripper_homing_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(_node->get_logger(), "Action server for gripper homing not available after waiting");
      rclcpp::shutdown();
    }

    this->_gripper_succeeded = false;
    auto goal_msg = GripperHomingAction::Goal();

    RCLCPP_INFO(_node->get_logger(), "Sending gripper goal - Homing gripper");

    auto send_goal_options = rclcpp_action::Client<GripperHomingAction>::SendGoalOptions();
    // send_goal_options.result_callback = std::bind(&PandaMove::gripper_status_callback, this, _1);
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<GripperHomingAction>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(_node->get_logger(), "Gripper homing succeeded");
            this->_gripper_succeeded = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(_node->get_logger(), "Gripper homing was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(_node->get_logger(), "Gripper homing was canceled");
            break;
        default:
            RCLCPP_ERROR(_node->get_logger(), "Unknown result code");
            break;
        }
        if (this->_gripper_succeeded != true){
            rclcpp::shutdown();
        }
    };
    this->_gripper_homing_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    rclcpp::sleep_for(std::chrono::seconds(10));
}

void PandaMove::go_home(const bool tmp_pose)
{
    // moveit::core::RobotStatePtr current_state = _group->getCurrentState(10);
    RCLCPP_INFO(_node->get_logger(), "Go Home!");
    _group->setPlannerId("PTP");
    std::vector<double> joint_group_position;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
    if (tmp_pose)
    {
        joint_group_position = {0.8858, -0.5503, 0.0642, -2.5330, 0.0108, 1.9932, 0.8437};
    }
    else
    {
        joint_group_position = {-0.0009, -0.7838, -0.0013, -2.3571, 0.0015, 1.5730, 0.7756};
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    _motion->joint_space_goal(joint_group_position, plan);

    _group->execute(plan);
}

void PandaMove::go_to_joint_angles()
{
    // moveit::core::RobotStatePtr current_state = _group->getCurrentState(10);
    RCLCPP_INFO(_node->get_logger(), "Going to temporary pose 2!");
    _group->setPlannerId("PTP");
    std::vector<double> joint_group_position;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
    joint_group_position = {1.4409, -0.5723, 0.2455, -2.550, 0.0128, 1.9931, 0.8646}; // trajectory beside hochregallager
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    _motion->joint_space_goal(joint_group_position, plan);
    _group->execute(plan);
}


void PandaMove::motionContraints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
{
    group->clearPathConstraints();
    moveit_msgs::msg::OrientationConstraint oCon;
    oCon.link_name = "panda_hand";
    oCon.header.frame_id = "panda_link0";
    oCon.orientation.x = 1.0;
    oCon.absolute_x_axis_tolerance = 0.1;
    oCon.absolute_y_axis_tolerance = 0.1;
    oCon.absolute_z_axis_tolerance = 0.1;
    oCon.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(oCon);
    group->setPathConstraints(constraints);
}

void PandaMove::visualMarkers(const geometry_msgs::msg::PoseStamped target_pose,
                             const moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task)
{
    _visual_tools->deleteAllMarkers();
    RCLCPP_INFO(_node->get_logger(), "Visualizing plan as trajectory line for %s task", task.c_str());
    const moveit::core::JointModelGroup *joint_model_group = _group->getCurrentState()->getJointModelGroup("panda_arm");
    _visual_tools->publishAxisLabeled(target_pose.pose, task.c_str());
    _visual_tools->publishText(_text_pose, task.c_str(), rvt::WHITE, rvt::XLARGE);
    _visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
    _visual_tools->trigger();
}

void PandaMove::motionExecution(geometry_msgs::msg::PoseStamped pose, const std::string task, const bool is_linear)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (is_linear)
    {
        pose.header.frame_id = "panda_link0";
        _motion->pose_goal(pose, plan, is_linear);
    }
    else
    {
        _motion->pose_goal(pose, plan, is_linear);
    }

    _group->execute(plan);
}

void PandaMove::pnpPipeLine(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place,
                           const double offset, const bool tmp_pose, const bool tmp_pose_2, const bool reverse)
{
    if (reverse)
    {
        if (tmp_pose_2)
            go_to_joint_angles();
    }

    // Pick
    RCLCPP_INFO(_node->get_logger(), "Panda Pre-Pick Pose");
    pick.pose.position.z += offset;
    motionExecution(pick, "Pre-Pick Pose", false);

    RCLCPP_INFO(_node->get_logger(), "Panda Pre-Pick Pose Gripper OPEN");
    this->open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));
    

    RCLCPP_INFO(_node->get_logger(), "Panda Pick Pose");
    pick.pose.position.z -= offset;
    motionExecution(pick, "Pick Pose", true);

    RCLCPP_INFO(_node->get_logger(), "Panda Pick Pose Gripper CLOSE");
    this->close_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "Panda Post Pick Pose");
    pick.pose.position.z += offset;
    std::cout << "PostPick Pose: " << pick.pose.position.z << std::endl;
    motionExecution(pick, "Post Pick Pose", true);

    if (!reverse)
    {
        if (tmp_pose)
            go_home(true);
        else
            go_home(false);
        if (tmp_pose_2)
            go_to_joint_angles();
    }
    else 
    {
        if (tmp_pose_2)
            go_to_joint_angles();
        if (tmp_pose)
            go_home(true);
        else
            go_home(false);
    }

    // Place
    RCLCPP_INFO(_node->get_logger(), "Panda Pre Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Pre Place Pose", false);

    RCLCPP_INFO(_node->get_logger(), "Panda Place Pose");
    place.pose.position.z -= offset;
    motionExecution(place, "Place Pose", true);

    RCLCPP_INFO(_node->get_logger(), "Panda Place Pose Gripper OPEN");
    this->open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "Panda Post Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Post Place Pose", true);
}

void PandaMove::pick_action(geometry_msgs::msg::PoseStamped pick, const double offset)
{
    RCLCPP_INFO(_node->get_logger(), "Panda Pre-Pick Pose");
    pick.pose.position.z += offset;
    motionExecution(pick, "Pre-Pick Pose", false);

    RCLCPP_INFO(_node->get_logger(), "Panda Pre-Pick Pose Gripper OPEN");
    this->open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "Panda Pick Pose");
    pick.pose.position.z -= offset;
    motionExecution(pick, "Pick Pose", true);

    RCLCPP_INFO(_node->get_logger(), "Panda Pick Pose Gripper CLOSE");
    this->close_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "Panda Post Pick Pose");
    pick.pose.position.z += offset;
    motionExecution(pick, "Post Pick Pose", true);
}

void PandaMove::place_action(geometry_msgs::msg::PoseStamped place, const double offset)
{
    // Place
    RCLCPP_INFO(_node->get_logger(), "Panda Pre Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Pre Place Pose", false);

    RCLCPP_INFO(_node->get_logger(), "Panda Place Pose");
    place.pose.position.z -= offset;
    motionExecution(place, "Place Pose", true);

    RCLCPP_INFO(_node->get_logger(), "Panda Pre-Pick Pose Gripper OPEN");
    this->open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "Panda Post Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Post Place Pose", true);
}

  void PandaMove::open_gripper()
  {
    // using namespace std::placeholders;

    if (!this->_gripper_motion_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(_node->get_logger(), "Action server for gripper motion not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = GripperCommandAction::Goal();
    goal_msg.command.position = 0.04;
    goal_msg.command.max_effort = 10;

    RCLCPP_INFO(_node->get_logger(), "Sending gripper goal - Opening gripper");

    auto send_goal_options = rclcpp_action::Client<GripperCommandAction>::SendGoalOptions();
    this->_gripper_motion_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void PandaMove::close_gripper()
  {
    // using namespace std::placeholders;

    if (!this->_gripper_motion_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(_node->get_logger(), "Action server for gripper motion not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = GripperCommandAction::Goal();
    goal_msg.command.position = 0.0;
    goal_msg.command.max_effort = 10;

    RCLCPP_INFO(_node->get_logger(), "Sending gripper goal - Closing gripper");

    auto send_goal_options = rclcpp_action::Client<GripperCommandAction>::SendGoalOptions();
    this->_gripper_motion_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

} // namespace iwtros2

// RCLCPP_COMPONENTS_REGISTER_NODE(iwtros2::PandaMove)
