/* Author: Andre Florindo*/

#include <vsl_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vsl_planner_descartes");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Main program
    vsl_motion_planning::VSLPlanner planner;

    planner.initRos();
    planner.initDescartes();

    pose_builder_client_ = nh.serviceClient<vsl_core::PoseBuilder>("single_course");
    vsl_core::PoseBuilder srv;
    // srv.request.num_layer = num_layer;
    // srv.request.num_course = num_course;
    // ROS_INFO_STREAM("Requesting pose in base frame: " << num_layer);
    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR("Could not localize service");
        return false;
    }
    ROS_INFO_STREAM("Service localized: ");

    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);

    marker_publisher_.publish(srv.response.single_course_marker);

    EigenSTL::vector_Isometry3d poses;
    Eigen::Isometry3d single_pose;
    poses.reserve(srv.response.single_course_poses.poses.size());

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_poses);
        poses.emplace_back(single_poses);
    }

    std::vector<descartes_core::TrajectoryPtPtr> input_traj;
    planner.generateTrajectory(poses, input_traj);

    std::vector<descartes_core::TrajectoryPtPtr> output_traj;
    planner.planPath(input_traj, output_traj);

    //ros::Duration(5).sleep();

    planner.runPath(output_traj);

    spinner.stop();

    return 0;
}