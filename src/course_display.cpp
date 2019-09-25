/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <course_display_topic.h>

namespace vsl_motion_planning
{

CourseDisplay::CourseDisplay() {}
CourseDisplay::~CourseDisplay() {}

void CourseDisplay::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("visualization/min_point_distance", config_.min_point_distance))
    {
        ROS_INFO_STREAM("Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load Topic parameters");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void CourseDisplay::getPoseArray(geometry_msgs::PoseArray &course_poses)
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

    ROS_INFO_STREAM("Service localized by Topic");

    course_poses = srv.response.single_course_poses;
}

void CourseDisplay::publishPosesMarkers(const geometry_msgs::PoseArray &course_poses)
{
    // creating rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;
p
    z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
    z_axes.ns = y_axes.ns = x_axes.ns = "axes";
    z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
    z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
    z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;
    z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

    // z properties
    z_axes.id = 0;
    z_axes.color.r = 0;
    z_axes.color.g = 0;
    z_axes.color.b = 1;
    z_axes.color.a = 1;

    // y properties
    y_axes.id = 1;
    y_axes.color.r = 0;
    y_axes.color.g = 1;
    y_axes.color.b = 0;
    y_axes.color.a = 1;

    // x properties
    x_axes.id = 2;
    x_axes.color.r = 1;
    x_axes.color.g = 0;
    x_axes.color.b = 0;
    x_axes.color.a = 1;

    // line properties
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.ns = "line";
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0);
    line.header.frame_id = config_.world_frame;
    line.scale.x = AXIS_LINE_WIDTH;
    line.id = 0;
    line.color.r = 1;
    line.color.g = 1;
    line.color.b = 0;
    line.color.a = 1;

    // creating axes markers

    int npoints = course_poses.poses.size();

    z_axes.points.reserve(2 * npoints);
    y_axes.points.reserve(2 * npoints);
    x_axes.points.reserve(2 * npoints);
    line.points.reserve(npoints);
    geometry_msgs::Point p_start, p_end;
    double distance = 0;

    EigenSTL::vector_Isometry3d poses;
    Eigen::Isometry3d single_pose;
    poses.reserve(npoints);

    for (unsigned int i = 0; i < npoints; i++)
    {
        tf::poseMsgToEigen(course_poses.poses[i], single_pose);
        poses.emplace_back(single_pose);
    }

    Eigen::Isometry3d prev = poses[0];
    for (unsigned int i = 0; i < npoints; i++)
    {
        const Eigen::Isometry3d &pose = poses[i];
        distance = (pose.translation() - prev.translation()).norm();

        tf::pointEigenToMsg(pose.translation(), p_start);

        if (distance > config_.min_point_distance)
        {
            Eigen::Isometry3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT, 0, 0);
            tf::pointEigenToMsg(moved_along_x.translation(), p_end);
            x_axes.points.emplace_back(p_start);
            x_axes.points.emplace_back(p_end);

            Eigen::Isometry3d moved_along_y = pose * Eigen::Translation3d(0, AXIS_LINE_LENGHT, 0);
            tf::pointEigenToMsg(moved_along_y.translation(), p_end);
            y_axes.points.emplace_back(p_start);
            y_axes.points.emplace_back(p_end);

            Eigen::Isometry3d moved_along_z = pose * Eigen::Translation3d(0, 0, AXIS_LINE_LENGHT);
            tf::pointEigenToMsg(moved_along_z.translation(), p_end);
            z_axes.points.emplace_back(p_start);
            z_axes.points.emplace_back(p_end);

            // saving previous
            prev = pose;
        }

        line.points.emplace_back(p_start);
    }

    markers_msg.markers.push_back(x_axes);
    markers_msg.markers.push_back(y_axes);
    markers_msg.markers.push_back(z_axes);
    markers_msg.markers.push_back(line);

    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);
    marker_publisher_.publish(markers_msg);
}

} // namespace vsl_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");

    vsl_motion_planning::CourseDisplay course_display;
    geometry_msgs::PoseArray course_poses;

    course_display.initTopic();
    course_display.getPoseArray(course_poses);
    course_display.publishPosesMarkers(course_poses);

    ros::spin();
}