/* Author: Andre Florindo*/

#include <cartesian_publisher.h>

namespace vsl_motion_planning
{

CartesianPublisher::CartesianPublisher() {}
CartesianPublisher::~CartesianPublisher() {}

void CartesianPublisher::initTopic()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    if (ph.getParam("tip_link", config_.tip_link) &&
        ph.getParam("base_link", config_.base_link))
    {
        ROS_INFO_STREAM("cartesian_publisher: Loaded Topic parameters");
    }
    else
    {
        ROS_ERROR_STREAM("cartesian_publisher: Failed to load Topic parameters");
        exit(-1);
    }

    cartesian_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>(CARTESIAN_TOPIC, 1, true);

    ROS_INFO_STREAM("cartesian_publisher: Task '" << __FUNCTION__ << "' completed");
}

void CartesianPublisher::startListener()
{
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped ee_cartesian_pose_msg;

    ros::Duration(1.0).sleep(); //wait until robot is initialized  

    while (nh_.ok())
    {
        try
        {
            listener.lookupTransform(config_.base_link, config_.tip_link,
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        tf::transformStampedTFToMsg(transform, ee_cartesian_pose_msg);
        cartesian_publisher_.publish(ee_cartesian_pose_msg);
    }

    ROS_INFO_STREAM("cartesian_publisher: Task '" << __FUNCTION__ << "' completed");
}

} // namespace vsl_motion_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_builder");

    vsl_motion_planning::CartesianPublisher cartesian_publisher;

    cartesian_publisher.initTopic();

    cartesian_publisher.startListener();

    ros::spin();
}