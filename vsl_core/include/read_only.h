// Inspired from setup_assistant_widget.h, header_wighet.h, start_screen_widget.h, setup_screen_widget.h
#ifndef READ_ONLY_H_
#define READ_ONLY_H_

// ROS
#include <ros/ros.h>

// C
#include <iostream>
#include <fstream>  

struct PathStruct
{
        std::vector<double> trivial_vector;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
};
    
bool getFileContent(const std::string &filename, std::vector<double> &newVector) //<-------------  Review


}



#endif
