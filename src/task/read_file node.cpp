/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <vsl_planner.h>
namespace vsl_motion_planning
{

void VSLPlanner::createCourse(CourseStruct &course, EigenSTL::vector_Isometry3d &poses)
{

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    //Read File containing the course
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/simplePath.txt", course);
    int npoints = course.x.size();
    poses.reserve(npoints);

    //Read File with the binormal and tangent of the course
    CourseStruct tangent;
    CourseStruct binormal;
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/tangent_simplePath.txt", tangent);
    readFileContent("/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/binormal_simplePath.txt", binormal);

    //determining orientation
    Eigen::Vector3d ee_z, ee_y, ee_x;
    Eigen::Isometry3d single_pose;

    for (unsigned int i = 0; i < npoints; i++)
    {
        ee_z << -binormal.x[i], -binormal.y[i], -binormal.z[i];

        ee_x << -tangent.x[i], -tangent.y[i], -tangent.z[i];
        ee_y = (ee_z.cross(ee_x)).normalized();

        Eigen::Isometry3d rot;
        rot.matrix() << ee_x(0), ee_y(0), ee_z(0), 0, ee_x(1), ee_y(1), ee_z(1), 0, ee_x(2), ee_y(2), ee_z(2), 0, 0, 0, 0, 1;
        //single_pose = Eigen::Translation3d(course.x[i]-0.8, course.y[i]+1.6, course.z[i]+0.8) * rot; //-0.8 1.6 0.8

        Eigen::Isometry3d rot_start_table;
        rot_start_table.matrix() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0 ,0 ,0, 1;
        single_pose = rot_start_table*(Eigen::Translation3d(course.x[i], course.y[i]-1.2-0.6, course.z[i]+0.78+0.002)) * rot; 

        poses.emplace_back(single_pose);
        
    }

    publishPosesMarkers(poses);

    ROS_INFO_STREAM("Task '" << __FUNCTION__ << "' completed");
    ROS_INFO_STREAM("Trajectory with " << npoints << " points was generated");
}

void VSLPlanner::readFileContent(std::string filename, CourseStruct &course)
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


void VSLPlanner::publishPosesMarkers(const EigenSTL::vector_Isometry3d &poses)
{
    // creating rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;
    visualization_msgs::MarkerArray markers_msg;

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
    z_axes.points.reserve(2 * poses.size());
    y_axes.points.reserve(2 * poses.size());
    x_axes.points.reserve(2 * poses.size());
    line.points.reserve(poses.size());
    geometry_msgs::Point p_start, p_end;
    double distance = 0;
    Eigen::Isometry3d prev = poses[0];
    for (unsigned int i = 0; i < poses.size(); i++)
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

    marker_publisher_.publish(markers_msg);
}
} // namespace vsl_motion_planning