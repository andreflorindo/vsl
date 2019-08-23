/* Author: Andre Florindo*/

#include <vsl_application.h>


int main(int argc, char **argv)
{
    // Start ROS Node                                            <----------
    ros::init(argc, argv, "main");

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    CourseStruct *course = new CourseStruct;

    const int result = getFileContent(course);

    //for (int i = 0; i < course->x.size(); i++)
    //{
      //  std::cout << course->x[i]<< std::endl;
    //}
    
    
    spinner.stop();

    return 0;
}