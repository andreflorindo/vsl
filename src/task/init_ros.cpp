#include <vsl_planner.h>

/* INIT ROS
  Goal:
    - Create a ros service client that will be used to send a robot path for execution.

  Hints:
*/

namespace vsl_motion_planning
{

VSLPlanner::VSLPlanner() {}
VSLPlanner::~VSLPlanner() {}

void VSLPlanner::initRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("group_name", config_.group_name) &&
        ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link) &&
        ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("trajectory/time_delay", config_.time_delay) &&
        ph.getParam("trajectory/foci_distance", config_.foci_distance) &&
        ph.getParam("trajectory/num_points", config_.num_points) &&
        ph.getParam("trajectory/center", config_.center) &&
        ph.getParam("trajectory/seed_pose", config_.seed_pose) &&
        ph.getParam("visualization/min_point_distance", config_.min_point_distance) &&
        nh.getParam("controller_joint_names", config_.joint_names))
    {
        ROS_INFO_STREAM("Loaded application parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load application parameters");
        exit(-1);
    }

    // creating publisher for trajectory visualization
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);

    typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;
    moveit_run_path_client_ptr_ = std::make_shared<client_type>(EXECUTE_TRAJECTORY_ACTION, true);

    // Establishing connection to server
    if (moveit_run_path_client_ptr_->waitForServer(ros::Duration(SERVER_TIMEOUT)))
    {
        ROS_INFO_STREAM("Connected to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '" << EXECUTE_TRAJECTORY_ACTION << "' action");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

}
