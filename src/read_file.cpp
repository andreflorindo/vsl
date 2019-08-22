/* Author: Andre Florindo*/
// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

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

bool getFileContent(CourseStruct *&course)
{
    //     https://stackoverflow.com/questions/46663046/save-read-double-vector-from-file-c                    //<-------------  Review
    //     std::vector<char> buffer{};
    //     std::istreambuf_iterator<char> iter(filename);
    //     std::istreambuf_iterator<char> end{};
    //     std::copy(iter, end, std::back_inserter(buffer));
    //     course->trivial_vector.resize(buffer.size() / sizeof(double));
    //     memcpy(&course->trivial_vector[0], &buffer[0], buffer.size());
    //     std::cout << course->trivial_vector.size()<< std::endl;
    //     for (int i = 0; i < buffer.size(); i++)
    //     {
    //         std::cout << buffer[i] << ' ';
    //     }
    //     for (int i = 0; i < course->trivial_vector.size(); i++)
    //     {
    //         std::cout << course->trivial_vector[i] << ' ';
    //     }

    // filename.close();

    std::ifstream infile{"/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl/examples/simplePath.txt", std::ios::in};

    if (!infile.good())
    {
        ROS_ERROR_NAMED("vsl", "Path as not able to be found");
        return false;
    }
    
    std::istream_iterator<double> infile_begin{infile};
    std::istream_iterator<double> eof{};
    std::vector<double> file_nums{infile_begin, eof};
    infile.close();

    int nx = 0;
    int ny = 0;
    int nz = 0;

    course->x.reserve(file_nums.size() / 3);
    course->y.reserve(file_nums.size() / 3);
    course->z.reserve(file_nums.size() / 3);

    for (int i = 0; i < file_nums.size(); i++)
    {
        if (i == nx * 3)
        {
            course->x.emplace_back(file_nums[i]);
            nx++;
        }
        else if (i == 1 + ny * 3)
        {
            course->y.emplace_back(file_nums[i]);
            ny++;       
        }
        else
            course->z.emplace_back(file_nums[i]);
        
    }

    return true;
};

int main(int argc, char **argv)
{
    // Start ROS Node                                            <----------
    ros::init(argc, argv, "read_only", ros::init_options::NoSigintHandler);

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    CourseStruct *course = new CourseStruct;

    const int result = getFileContent(course);
    
    // Shutdown ROS
    ros::shutdown();

    return 0;
}
