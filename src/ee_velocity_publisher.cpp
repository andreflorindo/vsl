/* Author: Andre Florindo*/

#include <ee_velocity_publisher.h>

namespace vsl_motion_planning
{

CartesianVelocityPublisher::CartesianVelocityPublisher() {}
CartesianVelocityPublisher::~CartesianVelocityPublisher() {}

void CartesianVelocityPublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link))
    {
        ROS_INFO_STREAM("ee_velocity_publisher: Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("ee_velocity_publisher: Failed to load Topic parameters");
        exit(-1);
    }

    boost::shared_ptr<trajectory_msgs::JointTrajectory const> sharedPtr;

    sharedPtr = ros::topic::waitForMessage<trajectory_msgs::JointTrajectory>("joint_path_command", nh_);
    if (sharedPtr == NULL)
    {
        ROS_ERROR_STREAM("ee_velocity_publisher: Failed to find topic joint_path_command");
        exit(-1);
    }
    else
    {
        joint_path_subscriber_ = nh.subscribe("joint_path_command", 1, &CartesianVelocityPublisher::subscriberCallback, this);
    }


    joint_request_publisher_ = nh.advertise<vsl_core::JointRequest>(EE_VELOCITY_TOPIC, 1, true);

    ROS_INFO_STREAM("ee_velocity_publisher: Task '" << __FUNCTION__ << "' completed");
}

void CartesianVelocityPublisher::subscriberCallback(const trajectory_msgs::JointTrajectory &msg)
{
    joint_path_ = msg;
}

void CartesianVelocityPublisher::publishJointRequest()
{
    //int num_joints = joint_path_.points[0].positions.size();
    // int num_points = joint_path_.points.size();

    // std::vector<std::vector<double>> buffer_position;
    // std::vector<std::vector<double>> buffer_velocity;
    // std::vector<std::vector<double>> buffer_acceleration;

    // buffer_position.resize(num_joints, std::vector<double>(num_points));
    // buffer_velocity.resize(num_joints, std::vector<double>(num_points));
    // buffer_acceleration.resize(num_joints, std::vector<double>(num_points));

    // vutput="screen">

    // futput="screen">
    // {utput="screen">
    //  utput="screen">++)
    //  utput="screen">
    //  utput="screen">_path_.points[j].positions[i];
    //  utput="screen">_path_.points[j].velocities[i];
    //  utput="screen">oint_path_.points[j].acceleration[i];
    //  utput="screen">
    // }utput="screen">

    // for (int j = 0; j < num_points; j++)
    // {
    //     ros::Time time(joint_path_.points[j].time_from_start.toSec());
    //     joint_request.header.seq = j;
    //     joint_request.header.stamp = time;
    //     joint_request.name = joint_path_.joint_names;
    //     joint_request.position = buffer_position[j];
    //     joint_request.velocity = buffer_velocity[j];
    //     joint_request.acceleration = buffer_acceleration[j];

    //     joint_request_publisher_.publish(joint_request);
    // }

    // ROS_INFO_STREAM("ee_velocity_publisher: Task '" << __FUNCTION__ << "' completed");
}

} // namespace vsl_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_velocity_publisher");

    vsl_motion_planning::CartesianVelocityPublisher ee_velocity_publisher;

    ee_velocity_publisher.initTopic();

    ee_velocity_publisher.publishJointRequest();

    ros::spin();
}