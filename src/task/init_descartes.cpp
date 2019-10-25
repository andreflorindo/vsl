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

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

// void VSLPlanner::loadRobotModel()
// {
//     robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
//     planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

//     /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
//                      the internal planning scene accordingly */
//     planning_scene_monitor_->startSceneMonitor();
//     /* listens to changes of world geometry, collision objects, and (optionally) octomaps
//                               world geometry, collision objects and optionally octomaps */
//     planning_scene_monitor_->startWorldGeometryMonitor();
//     /* listen to joint state updates as well as changes in attached collision objects
//                       and update the internal planning scene accordingly*/
//     planning_scene_monitor_->startStateMonitor();

//     kinematic_model_ = robot_model_loader_->getModel();
//     if (!kinematic_model_)
//     {
//         ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
//         exit(-1);
//     }

//     /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
//    for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
//    RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */

//     kinematic_state_.reset(new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));
    
//     /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
//    group is useful for dealing with one set of joints at a time such as a left arm or a end effector */

//     joint_model_group_ = kinematic_state_->getJointModelGroup(config_.group_name);

//     planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_, nh_, "planning_plugin", "request_adapters"));
// }

} // namespace vsl_motion_planning