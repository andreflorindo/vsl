// #include <ompl_vsl_planner.h>

// namespace vsl_motion_planning
// {

// void VSLPlanner::initOmpl()
// {
//     ros::NodeHandle nh;
//     ros::NodeHandle ph("~");

//     if (ph.getParam("group_name", config_.group_name) &&
//         ph.getParam("tip_link", config_.tip_link) &&
//         ph.getParam("base_link", config_.base_link) &&
//         ph.getParam("world_frame", config_.world_frame) &&
//         ph.getParam("trajectory/seed_pose", config_.seed_pose) &&
//         nh.getParam("controller_joint_names", config_.joint_names))
//     {
//         ROS_INFO_STREAM("Loaded application parameters");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to load application parameters");
//         exit(-1);
//     }

//     loadRobotModel();
//     createMotionPlanRequest()

//         ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
// }

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

//     kinematic_state_.reset(new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState()));

//     joint_model_group_ = kinematic_state_->getJointModelGroup(config_.group_name);

//     // Create pipeline
//     planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(kinematic_model_, nh_, "planning_plugin", "request_adapters"));
// }

// void VSLPlanner::getCourse(geometry_msgs::PoseArray &poses)
// {
//     // Initialize Service client
//     if (ros::service::waitForService(POSE_BUILDER_SERVICE, ros::Duration(SERVER_TIMEOUT)))
//     {
//         ROS_INFO_STREAM("Connected to '" << POSE_BUILDER_SERVICE << "' service");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to connect to '" << POSE_BUILDER_SERVICE << "' service");
//         exit(-1);
//     }

//     pose_builder_client_ = nh_.serviceClient<vsl_core::PoseBuilder>(POSE_BUILDER_SERVICE);
//     vsl_core::PoseBuilder srv;

//     if (!pose_builder_client_.call(srv))
//     {
//         ROS_ERROR_STREAM("Failed to call '" << POSE_BUILDER_SERVICE << "' service");
//         exit(-1);
//     }

//     poses = srv.response.single_course_poses;
// }

// void VSLPlanner::createMotionPlanRequest(geometry_msgs::PoseArray &poses)
// {
//     const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();
//     const std::vector<std::string> &link_model_names = joint_model_group_->getLinkModelNames();
//     ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

//     planning_interface::MotionPlanRequest req;
//     planning_interface::MotionPlanResponse res;

//     req.group_name = config_.group_name;

//     kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);
//     poses.header.frame_id = config_.group_name;
//     std::vector<double> goal_joint_values = {0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1};
//     robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
//     robot_state->update();
//     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
//     req.goal_constraints.push_back(joint_goal);

//     // Set joint tolerance
//     std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
//     for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
//     {
//         ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: " << goal_joint_constraint[x].position);
//         req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
//         req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
//     }

//     geometry_msgs::Pose pose_msg_goal;
//     const Eigen::Isometry3d &end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
//     pose_msg_goal = tf2::toMsg(end_effector_transform_goal);

//     // Solve the problem
//     // ========================================================================================
//     // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
//     // representation while planning
//     {
//         planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
//         /* Now, call the pipeline and check whether planning was successful. */
//         planning_pipeline->generatePlan(lscene, req, res);
//     }
//     /* Check that the planning was successful */
//     if (res.error_code_.val != res.error_code_.SUCCESS)
//     {
//         ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
//         return 0;
//     }

// }

// // void VSLPlanner::loadRobotModel()
// // {
// //     /* First, set the state in the planning scene to the final state of the last plan */

// //     kinematic_state_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentStateUpdated(response.trajectory_start);
// //     kinematic_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
// //     kinematic_state_::robotStateToRobotStateMsg(*kinematic_state_, req.start_state);

// //     robot_state::RobotState goal_state(*kinematic_state_);

// //     std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
// //     goal_state.setJointGroupPositions(joint_model_group, joint_values);
// //     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

// //     req.goal_constraints.clear();
// //     req.goal_constraints.push_back(joint_goal);

// // }

// } // namespace vsl_motion_planning

// int main(int argc, char** argv)
// {

//     ros::init(argc, argv, "ompl_vsl_planner");
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     // Main program
//     vsl_motion_planning::VSLPlanner planner;

//     planner.initOmpl();

//     geometry_msgs::PoseArray poses;
//     planner.getCourse(poses);

//     std::vector<descartes_core::TrajectoryPtPtr> input_traj;
//     planner.generateTrajectory(poses, input_traj);

//     std::vector<descartes_core::TrajectoryPtPtr> output_traj;
//     planner.planPath(input_traj, output_traj);

//     //ros::Duration(5).sleep();

//     planner.runPath(output_traj);

//     spinner.stop();

//     return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_vsl_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("group_name");
    move_group.setPlannerId(PLANNER_ID); //RRTConnect
    move_group.setPlanningTime(10.0f);
    move_group.setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING);

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