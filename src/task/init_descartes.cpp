#include <vsl_planner.h>

/* INIT DESCARTES
  Goal:
    - Initialize a Descartes RobotModel object for carrying out various robot related tasks.
    - Initialize a Descartes Path Planner for planning a robot path from a trajectory.
    - Use the moveit MoveGroup interface to move the arm to a pre-recorded positions saved in the moveit config package.
    - Verify that the arm reached the target.
*/

namespace vsl_motion_planning
{

void VSLPlanner::initDescartes()
{
    // Instantiating a robot model
    robot_model_ptr_.reset(new descartes_moveit::IkFastMoveitStateAdapter);
    robot_model_ptr_->setCheckCollisions(true);

    if (robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                     config_.group_name,
                                     config_.world_frame,
                                     config_.tip_link))
    {
        ROS_INFO_STREAM("Descartes Robot Model initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Robot Model");
        exit(-1);
    }

    bool succeeded = planner_.initialize(robot_model_ptr_);
    if (succeeded)
    {
        ROS_INFO_STREAM("Descartes Dense Planner initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to initialize Dense Planner");
        exit(-1);
    }

    // moving above-table

    moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
    move_group.setPlannerId(PLANNER_ID); //RRTConnect

    // setting above-table position as target
    if (!move_group.setNamedTarget(HOME_POSITION_NAME))
    {
        ROS_ERROR_STREAM("Failed to set home '" << HOME_POSITION_NAME << "' position");
        exit(-1);
    }

    moveit_msgs::MoveItErrorCodes result = move_group.move();
    if (result.val != result.SUCCESS)
    {
        ROS_ERROR_STREAM("Failed to move to " << HOME_POSITION_NAME << " position");
        exit(-1);
    }
    else
    {
        ROS_INFO_STREAM("Robot reached home position");
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}



} // namespace vsl_motion_planning