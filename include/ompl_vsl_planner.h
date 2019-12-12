#ifndef OMPL_VSL_PLANNER_H
#define OMPL_VSL_PLANNER_H

#include <ros/ros.h>
#include <pose_builder_server.h>
#include <const_ee_speed_time_parameterization.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

namespace vsl_motion_planning
{

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double SERVER_TIMEOUT = 5.0f; // seconds
const std::string HOME_POSITION_NAME = "above-table";

struct VSLPlannerConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
    std::string world_frame;
    std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
    std::vector<std::string> joint_names;
    std::string planner_id;
    double max_velocity_scaling;
};

class VSLPlanner
{
public:
    void initOmpl();
    void getCourse(std::vector<geometry_msgs::Pose> &poses);
    void createMotionPlanRequest(std::vector<geometry_msgs::Pose> &poses);
    

protected:
    void loadRobotModel();
    void addTimeParameterizationToOmpl(moveit_msgs::RobotTrajectory &traj);

protected:
    VSLPlannerConfiguration config_;
    ros::NodeHandle nh_;
    ros::ServiceClient pose_builder_client_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    mutable robot_state::RobotStatePtr kinematic_state_;
    const robot_model::JointModelGroup *joint_model_group_;
    robot_model::RobotModelConstPtr kinematic_model_;
    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    // planning_pipeline::PlanningPipelinePtr planning_pipeline_;
};

} // namespace vsl_motion_planning

#endif
