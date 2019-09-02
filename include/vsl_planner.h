/* Author: Andre Florindo*/

#ifndef VSL_PLANNER_H
#define VSL_PLANNER_H

// ROS
#include <ros/ros.h>
//#include <control_msgs/FollowJointTrajectoryAction.h>

// C
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <vector>
#include <eigen_conversions/eigen_msg.h>

// Action-Server
#include <actionlib/client/simple_action_client.h>

// MoveIt
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/MarkerArray.h>

//Descartes
#include <descartes_utilities/ros_conversions.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

struct CourseStruct
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

namespace vsl_motion_planning
{
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVER_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
// const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.01;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "above-table";
//Descartes

struct Configuration
{
  std::string group_name;
  std::string tip_link;
  std::string base_link;
  std::string world_frame;
  double time_delay; /* Time step between consecutive points in the robot path */
  //double foci_distance;           /* Controls the size of the curve */
  int num_points;                /* Number of points per curve */
  std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */
  double min_point_distance;     /* Minimum distance between consecutive trajectory points. */
  std::vector<std::string> joint_names;
};

class VSLPlanner
{
public:
  /*  Constructor
   *    Creates an instance of the application class
   */
  VSLPlanner();
  virtual ~VSLPlanner();

  /* Main Application Functions
   *  Functions that allow carrying out the various steps needed to run a
   *  plan and run application.  All these functions will be invoked from within
   *  the main routine.
   */

  void initRos();
  void initDescartes();
  void readFileContent(CourseStruct &course, EigenSTL::vector_Isometry3d &poses);
  void generateTrajectory(EigenSTL::vector_Isometry3d &poses, std::vector<descartes_core::TrajectoryPtPtr> &input_traj);
  void planPath(std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                std::vector<descartes_core::TrajectoryPtPtr> &output_path);
  void runPath(const std::vector<descartes_core::TrajectoryPtPtr> &path);

protected:
  void fromDescartesToMoveitTrajectory(const std::vector<descartes_core::TrajectoryPtPtr> &input_traj,
                                       trajectory_msgs::JointTrajectory &traj);
  void publishPosesMarkers(const EigenSTL::vector_Isometry3d &poses);
  void addVel(trajectory_msgs::JointTrajectory& traj);

protected:
  Configuration config_;
  ros::NodeHandle nh_;                                                                                              /* Object used for creating and managing ros application resources*/
  ros::Publisher marker_publisher_;                                                                                 /* Publishes visualization message to Rviz */
  std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> moveit_run_path_client_ptr_; /* Sends a robot trajectory to moveit for execution */
  //Descartes
  descartes_core::RobotModelPtr robot_model_ptr_;
  descartes_planner::SparsePlanner planner_;
};
} // namespace vsl_motion_planning

#endif