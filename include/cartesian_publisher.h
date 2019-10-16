/* Author: Andre Florindo*/
#ifndef CARTESIAN_POSE_TOPIC_H
#define CARTESIAN_POSE_TOPIC_H

// ROS
#include <ros/ros.h>

//TF
#include <tf/transform_listener.h>

namespace vsl_motion_planning
{

const std::string CARTESIAN_TOPIC = "cartesian_publisher";

struct CartesianPublisherConfiguration
{
    std::string tip_link;
    std::string base_link;
};

class CartesianPublisher
{
public:
    CartesianPublisher();
    virtual ~CartesianPublisher();

    void initTopic();
    void startListener();


protected:
    CartesianPublisherConfiguration config_;
    ros::NodeHandle nh_;
    ros::Publisher cartesian_publisher_;
};

} // namespace vsl_motion_planning

#endif