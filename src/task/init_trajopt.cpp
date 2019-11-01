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

void VSLPlanner::initTrajOpt()
{
    // Set the planner
    std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
    nh_.setParam("planning_plugin", planner_plugin_name);

    loadRobotModel();
    createMotionPlanRequest()

        ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void VSLPlanner::loadRobotModel()
{
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));

    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

    /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                     the internal planning scene accordingly */
    planning_scene_monitor_->startSceneMonitor();
    /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                              world geometry, collision objects and optionally octomaps */
    planning_scene_monitor_->startWorldGeometryMonitor();
    /* listen to joint state updates as well as changes in attached collision objects
                      and update the internal planning scene accordingly*/
    planning_scene_monitor_->startStateMonitor();

    kinematic_model_ = robot_model_loader_->getModel();

    if (!kinematic_model_)
    {
        ROS_ERROR_STREAM("Failed to load robot model from robot description parameter:robot_description");
        exit(-1);
    }

    kinematic_state_.reset(new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));

    joint_model_group_ = kinematic_state_->getJointModelGroup(config_.group_name);

    // Create pipeline
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_, nh_, "planning_plugin", "request_adapters"));
}

void VSLPlanner::createMotionPlanRequest()
{
    const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();
    const std::vector<std::string> &link_model_names = joint_model_group_->getLinkModelNames();
    ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);

    // geometry_msgs::Pose pose_msg_current;
    // const Eigen::Isometry3d &end_effector_transform_current = kinematic_state_->getGlobalLinkTransform(link_model_names.back());
    // pose_msg_current = tf2::toMsg(end_effector_transform_current);

    // std::vector<double> start_joint_values = {0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075};
    // robot_state->setJointGroupPositions(joint_model_group, start_joint_values);
    // robot_state->update();
    // geometry_msgs::Pose pose_msg_start;
    // const Eigen::Isometry3d &end_effector_transform_start = robot_state->getGlobalLinkTransform(link_model_names.back());
    // pose_msg_start = tf2::toMsg(end_effector_transform_start);

    // req.start_state.joint_state.name = joint_names;
    // req.start_state.joint_state.position = start_joint_values;
    // req.goal_constraints.clear();
    // req.group_name = config_.group_name;

    std::vector<double> goal_joint_values = {0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1};
    robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
    robot_state->update();
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
    req.goal_constraints.push_back(joint_goal);

    // Set joint tolerance
    std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
    for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
    {
        ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: " << goal_joint_constraint[x].position);
        req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
        req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
    }

    geometry_msgs::Pose pose_msg_goal;
    const Eigen::Isometry3d &end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
    pose_msg_goal = tf2::toMsg(end_effector_transform_goal);

    // Solve the problem
    // ========================================================================================
    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
        /* Now, call the pipeline and check whether planning was successful. */
        planning_pipeline->generatePlan(lscene, req, res);
    }
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
        return 0;
    }

}

// void VSLPlanner::loadRobotModel()
// {
//     /* First, set the state in the planning scene to the final state of the last plan */

//     kinematic_state_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentStateUpdated(response.trajectory_start);
//     kinematic_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
//     kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);

//     robot_state::RobotState goal_state(*kinematic_state_);

//     std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
//     goal_state.setJointGroupPositions(joint_model_group, joint_values);
//     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

//     req.goal_constraints.clear();
//     req.goal_constraints.push_back(joint_goal);

// }

} // namespace vsl_motion_planning