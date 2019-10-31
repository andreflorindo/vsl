/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <pose_builder_server.h>

namespace vsl_motion_planning
{

PoseBuilder::PoseBuilder() {}
PoseBuilder::~PoseBuilder() {}

void PoseBuilder::initServer()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("world_frame", config_.world_frame))
    {
        ROS_INFO_STREAM("pose_builder: Loaded Server parameters");
    }
    else
    {
        ROS_ERROR_STREAM("pose_builder: Failed to load Server parameters");
        exit(-1);
    }

    ROS_INFO_STREAM("pose_builder: Task '" << __FUNCTION__ << "' completed");
}

void PoseBuilder::createCourse()
{
    //Read File containing the course
    CourseStruct course;
    //std::shared_ptr<CourseStruct> course = std::make_shared<CourseStruct>();
    //readFileContent("/home/amendesflorind/workspaces/kuka_testing_ws/src/vsl_core/examples/simplePath.txt", course);
    readFileContent("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_core/examples/simplePath.txt", course);
    int npoints = course.x.size();

    //Read Files with the binormal and tangent of the course
    CourseStruct tangent;
    CourseStruct binormal;
    readFileContent("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_core/examples/tangent_simplePath.txt", tangent);
    readFileContent("/home/andreflorindo/workspaces/vsl_motion_planner_ws/src/vsl_core/examples/binormal_simplePath.txt", binormal);

    //Initializate pose message
    course_poses.poses.reserve(npoints); //  <---------------
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
        //single_pose = rot_start_table * (Eigen::Translation3d(course.x[i], course.y[i] - 1.2 - 0.6, course.z[i] + 0.78 + 0.002)) * rot;
        single_pose = rot_start_table * (Eigen::Translation3d(course.x[i]- 1.2 - 0.6, course.y[i] - 0.6, course.z[i] + 0.78 + 0.002+0.7)) * rot;

        tf::poseEigenToMsg(single_pose, single_pose_msg);

        course_poses.poses.emplace_back(single_pose_msg);
    }
    pose_builder_server_ = nh_.advertiseService(POSE_BUILDER_SERVICE, &PoseBuilder::serviceCallback, this); //  <---------------

    ROS_INFO_STREAM("pose_builder: Task '" << __FUNCTION__ << "' completed");
    ROS_INFO_STREAM("pose_builder: Trajectory with " << npoints << " points was generated");
}

void PoseBuilder::readFileContent(std::string filename, CourseStruct &course)
{
    std::ifstream infile{filename, std::ios::in};

    if (!infile.good())
    {
        ROS_ERROR_STREAM("pose_builder: Path as not able to be found. Trajectory generation failed");
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

bool PoseBuilder::serviceCallback(vsl_core::PoseBuilder::Request &request, vsl_core::PoseBuilder::Response &response)
{
    response.single_course_poses = course_poses; //  <---------------
    return true;
}

} // namespace vsl_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");

    vsl_motion_planning::PoseBuilder pose_builder;

    pose_builder.initServer();

    pose_builder.createCourse();

    ros::spin();
}