/* Author: Andre Florindo*/

#ifndef POSE_BUILDER_H
#define POSE_BUILDER_H

// ROS
#include <ros/ros.h>
#include <vsl_core/PoseBuilder.h>

// C
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

// MoveIt
#include <visualization_msgs/MarkerArray.h>

struct CourseStruct
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

namespace vsl_motion_planning
{
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.01;

struct PoseBuilderConfiguration
{
    std::string world_frame;
    double min_point_distance;     /* Minimum distance between consecutive trajectory points. */
};

class PoseBuilder
{
public:
  PoseBuilder();
  virtual ~PoseBuilder();
  void createCourse();
  geometry_msgs::PoseArray course_poses;
  visualization_msgs::MarkerArray markers_msg;

protected:
  void readFileContent(std::string &filename, CourseStruct &course);
  void publishPosesMarkers(const EigenSTL::vector_Isometry3d &poses, const int &npoints);

protected:
  PoseBuilderConfiguration config_;
  ros::NodeHandle nh_;                                                                                                                                                                           
  ros::ServiceServer pose_builder_server_;

};

} 
#endif