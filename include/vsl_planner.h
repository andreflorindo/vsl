/* Author: Andre Florindo*/

#ifndef VSL_PLANNER_H
#define VSL_PLANNER_H

// ROS
#include <ros/ros.h>
#include <pose_builder_server.h>
#include <course_display_topic.h>
#include <ee_cartesian_pose_publisher.h>
#include <ee_velocity_publisher.h>

// Eigen library
#include <eigen_conversions/eigen_msg.h>

// Action-Server
#include <actionlib/client/simple_action_client.h>

// MoveIt
#include <moveit_msgs/ExecuteTrajectoryAction.h>    //or #include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Descartes
#include <descartes_utilities/ros_conversions.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/sparse_planner.h>
//#include <descartes_planner/dense_planner.h>

namespace vsl_motion_planning
{
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";
const double SERVER_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "above-table";
const std::string JOINT_POSE_TOPIC = "joint_pose";
const double MAX_VELOCITY_SCALING = 0.1;

struct VSLPlannerConfiguration
{
    std::string group_name;
    std::string tip_link;
    std::string base_link;
    std::string world_frame;
    // double time_delay; /* Time step between consecutive points in the robot path */
    std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
    std::vector<std::string> joint_names;
};

class VSLPlanner
{
public:
    VSLPlanner();
    virtual ~VSLPlanner();

    void initRos();
    void initDescartes();
    bool getCourse(EigenSTL::vector_Isometry3d &poses);
    void generateTrajectory(EigenSTL::vector_Isometry3d &poses, std::vector<descartes_core::TrajectoryPtPtr> &input_traj);
    void planPath(std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                  std::vector<descartes_core::TrajectoryPtPtr> &output_path);
    void runPath(const std::vector<descartes_core::TrajectoryPtPtr> &path);

protected:
    void fromDescartesToMoveitTrajectory(const std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                                         trajectory_msgs::JointTrajectory &traj);
    void addVel(trajectory_msgs::JointTrajectory &traj);
    void addAcc(trajectory_msgs::JointTrajectory &traj);
    void computeToolVel();

protected:
    VSLPlannerConfiguration config_;
    ros::NodeHandle nh_;
    std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> moveit_run_path_client_ptr_; /* Sends a robot trajectory to moveit for execution */
    ros::ServiceClient pose_builder_client_;
    ros::Publisher joint_pose_publisher_; 

    //Descartes
    descartes_core::RobotModelPtr robot_model_ptr_;
    descartes_planner::SparsePlanner planner_;
    //descartes_planner::DensePlanner planner_;

};

} // namespace vsl_motion_planning
#endif