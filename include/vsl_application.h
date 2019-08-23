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


struct CourseStruct
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

bool getFileContent(CourseStruct *&course);

#endif