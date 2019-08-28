/* Author: Andre Florindo*/

#include <vsl_planner.h>

int main(int argc, char **argv)
{
    // Start ROS Node                                            <----------
    ros::init(argc, argv, "main");

    // ROS Spin
    ros::AsyncSpinner spinner(2);
    spinner.start();

    vsl_motion_planning::VSLPlanner planner;
    
    planner.initRos();

    planner.initDescartes();

    CourseStruct *course = new CourseStruct;

    const int result = getFileContent(course);

    /*for (int i = 0; i < course->x.size(); i++)
    {
        std::cout << course->y[i]<< std::endl;
    }
    */
    
    spinner.stop();

    return 0;
}