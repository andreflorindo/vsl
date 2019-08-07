#include <vsl_sreen_window.h>




#include "widgets/setup_assistant_widget.h"
#include <ros/ros.h>
#include <QApplication>
#include <QMessageBox>
#include <boost/program_options.hpp>
#include <signal.h>
#include <locale.h>

static void siginthandler(int param)
{
  QApplication::quit();
}

void usage(boost::program_options::options_description& desc, int exit_code)
{
  std::cout << desc << std::endl;
  exit(exit_code);
}






int main(int argc, char **argv)
{
  // Parse parameters
  namespace po = boost::program_options;

  // Declare the supported options
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Show help message")("debug,g", "Run in debug/test mode")(
      "urdf_path,u", po::value<std::string>(), "Optional, path to URDF file in ROS package")(
      "config_pkg,c", po::value<std::string>(), "Optional, pass in existing config package to load");

  // Process options
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
      usage(desc, 0);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    usage(desc, 1);
  }







    // Start ROS Node                       <----------
    ros::init(argc, argv, "moveit_setup_assistant", ros::init_options::NoSigintHandler);

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Create Qt Application
    QApplication qt_app(argc, argv);
    // numeric values should always be POSIX
    setlocale(LC_NUMERIC, "C");

    // Load Qt Widget
    moveit_setup_assistant::SetupAssistantWidget saw(nullptr, vm);
    saw.setMinimumWidth(980);
    saw.setMinimumHeight(550);
    //  saw.setWindowState( Qt::WindowMaximized );

    saw.show();

    signal(SIGINT, siginthandler);

    // Wait here until Qt App is finished
    const int result = qt_app.exec();

    // Shutdown ROS
    ros::shutdown();

    return result;
}