#ifndef ARM_UTILITIES_HPP
#define ARM_UTILITIES_HPP

#include <iostream>
#include <string>

namespace iwtros2
{

/**
 * @brief Robot Basic Parameters
 *
 */
class robot_config
{
  public:
    std::string ARM_GROUP_NAME;
    std::string ARM_REFERENCE_FRAME;
    std::string ARM_END_EFFECTOR;
    std::string PIPELINE_ID;
    std::string PTP_PLANNER_ID;
    std::string LIN_PLANNER_ID;
    double MAX_VEL_SCALING;
    double MAX_ACE_SCALING;

    robot_config()
    {
        ARM_GROUP_NAME = "panda_arm";
        ARM_REFERENCE_FRAME = "panda_link0";
        ARM_END_EFFECTOR = "panda_hand";
        PIPELINE_ID = "pilz";
        PTP_PLANNER_ID = "PTP";
        LIN_PLANNER_ID = "LIN";
        MAX_VEL_SCALING = 0.3;
        MAX_ACE_SCALING = 0.25;
    }
};

} // namespace iwtros2

#endif // ARM_UTILITIES_HPP
