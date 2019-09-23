/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <pose_builder.h>

namespace vsl_motion_planning
{

PoseBuilder::PoseBuilder() {}
PoseBuilder::~PoseBuilder() {}

void PoseBuilder::initServer()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("world_frame", config_.world_frame) &&
        ph.getParam("visualization/min_point_distance", config_.min_point_distance))
    {
        ROS_INFO_STREAM("Loaded Server parameters");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load Server parameters");
        exit(-1);
    }

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
}

void PoseBuilder::createCourse()
{
    //Read File containing the course
    CourseStruct course;
    //std::shared_ptr<CourseStruct> course = std::make_shared<CourseStruct>();
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/simplePath.txt", course);
    int npoints = course.x.size();

    //Read Files with the binormal and tangent of the course
    CourseStruct tangent;
    CourseStruct binormal;
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/tangent_simplePath.txt", tangent);
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/binormal_simplePath.txt", binormal);

    //Initializate pose message
    course_poses.poses.reserve(npoints);                                //  <---------------
    course_poses.header.frame_id = config_.world_frame;

    //determining orientation and calculate pose
    Eigen::Vector3d ee_z, ee_y, ee_x;
    Eigen::Isometry3d single_pose;
    geometry_msgs::Pose single_pose_msg;

    for (unsigned int i = 0; i < npoints; i++)
    {
        ee_z << -binormal.x[i], -binormal.y[i], -binormal.z[i];

        ee_x << -tangent.x[i], -tangent.y[i], -tangent.z[i];
        ee_y = (ee_z.cross(ee_x)).normalized();

        Eigen::Isometry3d rot;
        rot.matrix() << ee_x(0), ee_y(0), ee_z(0), 0, ee_x(1), ee_y(1), ee_z(1), 0, ee_x(2), ee_y(2), ee_z(2), 0, 0, 0, 0, 1;
        //single_pose = Eigen::Translation3d(course.x[i]-0.8, course.y[i]+1.6, course.z[i]+0.8) * rot; //-0.8 1.6 0.8

        Eigen::Isometry3d rot_start_table;
        rot_start_table.matrix() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        single_pose = rot_start_table * (Eigen::Translation3d(course.x[i], course.y[i] - 1.2 - 0.6, course.z[i] + 0.78 + 0.002)) * rot;

        tf::poseEigenToMsg(single_pose, single_pose_msg);

        course_poses.poses.emplace_back(single_pose_msg);
    }

    publishPosesMarkers(course_poses, npoints);
    pose_builder_server_ = nh_.advertiseService("single_course", &PoseBuilder::serviceCallback, this);                   //  <---------------

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
    ROS_INFO_STREAM("Trajectory with " << npoints << " points was generated");
}

void PoseBuilder::readFileContent(std::string filename, CourseStruct &course)
{
    std::ifstream infile{filename, std::ios::in};

    if (!infile.good())
    {
        ROS_ERROR_STREAM("Path as not able to be found. Trajectory generation failed");
        exit(-1);
    }

    std::istream_iterator<double> infile_begin{infile};
    std::istream_iterator<double> eof{};
    std::vector<double> file_nums{infile_begin, eof};
    infile.close();

    int nx = 0;
    int ny = 0;
    int npoints = file_nums.size() / 3;

    course.x.reserve(npoints);
    course.y.reserve(npoints);
    course.z.reserve(npoints);

    for (int i = 0; i < file_nums.size(); i++)
    {
        if (i == nx * 3)
        {
            course.x.emplace_back(file_nums[i]);
            nx++;
        }
        else if (i == 1 + ny * 3)
        {
            course.y.emplace_back(file_nums[i]);
            ny++;
        }
        else
            course.z.emplace_back(file_nums[i]);
    }
}

void PoseBuilder::publishPosesMarkers(const geometry_msgs::PoseArray &course_poses, const int& npoints)
{
    // creating rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;

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


        // creating publisher for trajectory visualization
            //marker_publisher_.publish(markers_msg);
    // marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC, 1, true);
}

bool PoseBuilder::serviceCallback(vsl_core::PoseBuilder::Request &request, vsl_core::PoseBuilder::Response &response)
{
    response.single_course_poses = course_poses;                                            //  <---------------
    response.single_course_marker = markers_msg;                                           //  <---------------

    return true;
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    vsl_motion_planning::PoseBuilder posebuilder;

    posebuilder.initServer();

    posebuilder.createCourse();

    ros::Duration(30).sleep();

}