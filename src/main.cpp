// #include <iwtros2_launch/gripper_controller.hpp>
#include <panda_manipulation.hpp>
#include <arm_utilities.hpp>

// using GripperCommand = control_msgs::action::GripperCommand;
// using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("panda_main_node", options);
    auto node_g = rclcpp::Node::make_shared("plc_control_sub_pub_node", options);

    // node->declare_parameter("planning_plugin", "pilz"); //###########

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto executor_g = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread([&executor]() { executor->spin(); }).detach();
    executor_g->add_node(node_g);

    // Setup Move group planner
    auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_main_node_logger");

    iwtros2::robot_config conf;

    group->setPlanningPipelineId(conf.PIPELINE_ID);
    group->setPlannerId(conf.PTP_PLANNER_ID);
    group->setMaxVelocityScalingFactor(conf.MAX_VEL_SCALING);
    group->setMaxAccelerationScalingFactor(conf.MAX_ACE_SCALING);
    group->setPoseReferenceFrame(conf.ARM_REFERENCE_FRAME);
    group->setEndEffectorLink(conf.ARM_END_EFFECTOR);
    // group->allowReplanning(true);

    // Reference frame for this robot
    RCLCPP_INFO(LOGGER, "Planning frame: %s", group->getPlanningFrame().c_str());

    // Name of the end-effector link for this group
    RCLCPP_INFO(LOGGER, "End effector link: %s", group->getEndEffectorLink().c_str());

    // List of all the groups in the robot
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(group->getJointModelGroupNames().begin(), group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
    
    RCLCPP_INFO(LOGGER, "Default Planning Pipeline Id: %s", group->getDefaultPlanningPipelineId().c_str());
    RCLCPP_INFO(LOGGER, "Planning Pipeline Id: %s", group->getPlanningPipelineId().c_str());
    RCLCPP_INFO(LOGGER, "Default Planner Id: %s", group->getDefaultPlannerId().c_str());


    auto panda_move = std::make_shared<iwtros2::PandaMove>(node, group);
    auto plc_contl = std::make_shared<iwtros2::ControlPLC>(node_g);

    geometry_msgs::msg::PoseStamped table_pose_0_place = 
        // panda_move->generatePose(-0.141, 0.585, 1.265, M_PI, 0, 3 * M_PI / 4, "panda_link0");
        panda_move->generatePose(-0.130, 0.390, 0.155, -3.135, 0.002, -3.140, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_1_place = 
        panda_move->generatePose(0.047, 0.387, 0.157, -3.133, 0.038, -3.139, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_2_place = 
        panda_move->generatePose(0.053, 0.590, 0.156, -3.138, 0.021, 3.137, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_3_place = 
        panda_move->generatePose(-0.131, 0.592, 0.154, 3.138, 0.036, 3.141, "panda_link0");

    geometry_msgs::msg::PoseStamped table_pose_0_pick = table_pose_0_place;
        // panda_move->generatePose(-0.141, 0.5850, 1.260, M_PI, 0, 3 * M_PI / 4, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_1_pick = table_pose_1_place;
        // panda_move->generatePose(0.0530, 0.5885, 1.2615, M_PI, 0, 3 * M_PI / 4, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_2_pick = table_pose_2_place;
        // panda_move->generatePose(0.0500, 0.7590, 1.266, M_PI, 0, 3 * M_PI / 4, "panda_link0");
    geometry_msgs::msg::PoseStamped table_pose_3_pick = table_pose_3_place;
        // panda_move->generatePose(-0.1423, 0.7590, 1.262, M_PI, 0, 3 * M_PI / 4, "panda_link0");
        
    geometry_msgs::msg::PoseStamped home_pose =
        // panda_move->generatePose(0.5, 0, 1.65896, -M_PI, 0, M_PI, "panda_link0");
        // panda_move->generatePose(0.492, 0.052, 0.363, -2.889, 0.059, -0.882, "panda_link0");
        panda_move->generatePose(0.308, -0.001, 0.590, -3.139, -0.002, -0.007, "panda_link0");
    geometry_msgs::msg::PoseStamped conveyor_pose =
        // panda_move->generatePose(0.235, -0.43, 1.263, M_PI, 0, M_PI / 4, "panda_link0"); // default
        // panda_move->generatePose(0.2360, -0.4298, 1.263, M_PI, 0, M_PI / 4, "panda_link0");
        panda_move->generatePose(0.174, -0.432, 0.174, 3.139, -0.022, -1.575, "panda_link0");
    geometry_msgs::msg::PoseStamped hochregallager_pose =
        panda_move->generatePose(0.555, 0.069, 1.345, M_PI, 0, 3 * M_PI / 4, "panda_link0"); 
    geometry_msgs::msg::PoseStamped loading_pose =
        panda_move->generatePose(0.0, 0.5, 1.245, M_PI, 0, 3 * M_PI / 4, "panda_link0");

    geometry_msgs::msg::PoseStamped slot_pose;
    bool use_table = true;
    

    rclcpp::Rate rate(1);
    executor_g->spin_once();
    while (rclcpp::ok())
    {
        if (plc_contl->conveyor_pick){
            switch (plc_contl->slot_id)
            {
                case 0:
                    slot_pose = table_pose_0_place;
                    break;
                case 1:
                    slot_pose = table_pose_1_place;
                    break;
                case 2:
                    slot_pose = table_pose_2_place;
                    break;
                case 3:
                    slot_pose = table_pose_3_place;
                    break;
                default:
                    RCLCPP_ERROR(LOGGER, "Invalid slot ID!");
                    plc_contl->conveyor_pick = false;
                    plc_contl->table_pick = false;
            }
        }

        if (plc_contl->table_pick){
            switch (plc_contl->slot_id)
            {
                case 0:
                    slot_pose = table_pose_0_pick;
                    break;
                case 1:
                    slot_pose = table_pose_1_pick;
                    break;
                case 2:
                    slot_pose = table_pose_2_pick;
                    break;
                case 3:
                    slot_pose = table_pose_3_pick;
                    break;
                default:
                    RCLCPP_ERROR(LOGGER, "Invalid slot ID!");
                    plc_contl->conveyor_pick = false;
                    plc_contl->table_pick = false;
            }
        }

        if (plc_contl->move_home)
        {
            panda_move->go_home(false);
            plc_contl->move_home = false;
            plc_contl->plc_publish(true, false, false, false, false, use_table);
        }
        if (plc_contl->conveyor_pick)
        {
            // std::cout<<"slot ID: "<< plc_contl->slot_id<<std::endl;
            // RCLCPP_INFO(rclcpp::get_logger("panda_main_node"), "Slot ID: "+std::to_string(plc_contl->slot_id>));
            if (!use_table)
            {
                panda_move->pnpPipeLine(conveyor_pose, hochregallager_pose, 0.15, false, true, false); //hochregallager_pose
                plc_contl->conveyor_pick = false;
                plc_contl->plc_publish(false, false, false, true, false, use_table); // Placed the product on Hochregallager
            }
            else
            {
                panda_move->pnpPipeLine(conveyor_pose, slot_pose, 0.15, false, true, false);
                plc_contl->conveyor_pick = false;
                plc_contl->plc_publish(false, false, false, false, true, use_table); // Placed the product on table
            }
        }
        if (plc_contl->table_pick)
        {
            // std::cout<<"slot ID: "<< plc_contl->slot_id<<std::endl;
            // RCLCPP_INFO(LOGGER, "Slot ID: "+std::to_string(plc_contl->slot_id>));
            panda_move->pnpPipeLine(slot_pose, conveyor_pose, 0.15, false, true, true);
            plc_contl->table_pick = false;
            plc_contl->plc_publish(false, false, true, false, false, use_table); // Placed the product on conveyor belt
        }

        if (plc_contl->hochregallager_pick)
        {
            panda_move->pick_action(hochregallager_pose, 0.15);
            plc_contl->plc_publish(false, true, false, false, false, use_table);
            panda_move->go_home(false);
            panda_move->place_action(conveyor_pose, 0.15);
            // panda_move->pnpPipeLine(hochregallager_pose, conveyor_pose, 0.15, false, false);
            plc_contl->hochregallager_pick = false;
            plc_contl->plc_publish(false, false, true, false, false, use_table); // Placed the product on conveyor belt.
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Doing nothing and I am Sad!");
        }

        // executor->spin_once();
        executor_g->spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
