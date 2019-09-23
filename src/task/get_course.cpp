#include <vsl_planner.h>

namespace vsl_motion_planning
{
bool VSLPlanner::getCourse(EigenSTL::vector_Isometry3d &poses)
{
    pose_builder_client_ = nh_.serviceClient<vsl_core::PoseBuilder>("single_course");
    vsl_core::PoseBuilder srv;
    // srv.request.num_layer = num_layer;
    // srv.request.num_course = num_course;
    // ROS_INFO_STREAM("Requesting pose in base frame: " << num_layer);
    if (!pose_builder_client_.call(srv))
    {
        ROS_ERROR("Could not localize service");
        exit(-1);
    }

    ROS_INFO_STREAM("Service localized ");
    
    Eigen::Isometry3d single_pose;
    poses.reserve(srv.response.single_course_poses.poses.size());

    for (unsigned int i = 0; i < srv.response.single_course_poses.poses.size(); i++)
    {
        tf::poseMsgToEigen(srv.response.single_course_poses.poses[i], single_pose);
        poses.emplace_back(single_pose);
    }
}
} // namespace vsl_motion_planning