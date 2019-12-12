// #ifndef OMPL_VSL_PLANNER_H
// #define OMPL_VSL_PLANNER_H

// #include <pluginlib/class_loader.h>
// #include <ros/ros.h>

// // MoveIt
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <boost/scoped_ptr.hpp>

// namespace vsl_motion_planning
// {

// const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
// const double SERVER_TIMEOUT = 5.0f; // seconds
// const std::string POSE_BUILDER_SERVICE = "single_course";

// struct VSLPlannerConfiguration
// {
//     std::string group_name;
//     std::string tip_link;
//     std::string base_link;
//     std::string world_frame;
//     std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
//     std::vector<std::string> joint_names;
// };

// class VSLPlanner
// {
// public:
//     void initOmpl();
//     void getCourse(geometry_msgs::PoseArray &poses);
//     void createMotionPlanRequest(geometry_msgs::PoseArray &poses);

// protected:
//     void loadRobotModel();

// protected:
//     VSLPlannerConfiguration config_;
//     ros::NodeHandle nh_;
//     ros::ServiceClient pose_builder_client_;

//     robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
//     mutable robot_state::RobotStatePtr kinematic_state_;
//     const robot_model::JointModelGroup *joint_model_group_;
//     robot_model::RobotModelConstPtr kinematic_model_;
//     planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
//     planning_pipeline::PlanningPipelinePtr planning_pipeline_;
// }

// } // namespace vsl_motion_planning

// #endif

#ifndef OMPL_VSL_PLANNER_H
#define OMPL_VSL_PLANNER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>