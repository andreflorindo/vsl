/* Author: Andre Florindo*/

#ifndef POSE_BUILDER_SERVER_H
#define POSE_BUILDER_SERVER_H

// ROS
#include <ros/ros.h>
#include <vsl_core/PoseBuilder.h>

// C++
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <vector>

// Eigen library
#include <eigen_conversions/eigen_msg.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

struct CourseStruct
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

namespace vsl_motion_planning
{

const std::string POSE_BUILDER_SERVICE = "single_course";

struct PoseBuilderConfiguration
{
    std::string world_frame;
};

class PoseBuilder
{
public:
  PoseBuilder();
  virtual ~PoseBuilder();
  void createCourse();
  geometry_msgs::PoseArray course_poses;

  void initServer();
  bool serviceCallback(vsl_core::PoseBuilder::Request &request, vsl_core::PoseBuilder::Response &response);

protected:
  void readFileContent(std::string filename, CourseStruct &course);
  

protected:
  PoseBuilderConfiguration config_;
  ros::NodeHandle nh_;
  ros::ServiceServer pose_builder_server_;                                                                                                                                                                        

};

} 
#endif