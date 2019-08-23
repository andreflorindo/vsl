/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <vsl_application.h>

bool getFileContent(CourseStruct *&course)
{
    //     https://stackoverflow.com/questions/46663046/save-read-double-vector-from-file-c                    //<-------------  Other way

    std::ifstream infile{"/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/simplePath.txt", std::ios::in};

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
    ROS_INFO("File found, reading complete");

    return true;

};


