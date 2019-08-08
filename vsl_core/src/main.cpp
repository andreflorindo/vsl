// Autor: Andre Florindo


#include <read_only.h>

int main(int argc, char **argv)
{
    // Start ROS Node                                            <----------
    ros::init(argc, argv, "read_only", ros::init_options::NoSigintHandler);

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    PathStruct path;

    // Wait here until Qt App is finished
    const int result = getFileContent(PathStruct& path);  //<-------------  Review;

    std::cin << path.trivial_vector << std::cout;

    // Shutdown ROS
    ros::shutdown();

    return result;
}