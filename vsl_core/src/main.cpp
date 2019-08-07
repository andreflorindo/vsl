#include <vsl_sreen_window.h>

int main(int argc, char **argv)
{
    // Start ROS Node                                            <----------
    ros::init(argc, argv, "vsl_screen_window", ros::init_options::NoSigintHandler);

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Create Qt Application
    QApplication qt_app(argc, argv);
    vsl_screen_window::SetupAssistantWidget saw(nullptr);
    saw.setMinimumWidth(980);
    saw.setMinimumHeight(550);
    saw.show();

    // Wait here until Qt App is finished
    const int result = qt_app.exec();

    // Shutdown ROS
    ros::shutdown();

    return result;
}