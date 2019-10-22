/* Author: Andre Florindo*/
#ifndef EE_VELOCITY_PUBLISHER_H
#define EE_VELOCITY_PUBLISHER_H

// ROS
#include <ros/ros.h>

//msg
#include <vsl_core/JointRequest.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace vsl_motion_planning
{

//const std::string EE_VELOCITY_TOPIC = "ee_velocity";

struct CartesianVelocityPublisherConfiguration
{
    std::string tip_link;
    std::string base_link;
};

class CartesianVelocityPublisher
{
public:
    CartesianVelocityPublisher();
    virtual ~CartesianVelocityPublisher();

    void initTopic();
    void subscriberCallback(const trajectory_msgs::JointTrajectory &msg);
    void checkForPublisher();
    void publishJointRequest();

    trajectory_msgs::JointTrajectory joint_path_;
    int total_num_points = 0;
    int seq = 0;
    ros::Time time_point;

protected:
    CartesianVelocityPublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher joint_request_publisher_;
    ros::Subscriber joint_path_subscriber_;
};

} // namespace vsl_motion_planning

#endif