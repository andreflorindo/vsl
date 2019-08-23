/* Author: Andre Florindo*/

#ifndef VSL_APPLICATION_H
#define VSL_APPLICATION_H

// ROS
#include <ros/ros.h>

// C
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <vector>



// Moveit

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



struct CourseStruct
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

bool getFileContent(CourseStruct *&course);

#endif