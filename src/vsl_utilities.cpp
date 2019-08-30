#include <vsl_planner.h>
#include <descartes_utilities/ros_conversions.h>

namespace vsl_motion_planning
{



void swap_segments(EigenSTL::vector_Isometry3d& poses, unsigned npoints, unsigned idx1, unsigned idx2)
{
  auto n = npoints / 2;
  std::swap_ranges(poses.begin() + n * idx1, poses.begin() + n * (idx1 + 1),
                   poses.begin() + n * idx2);
}

void insert_segment(EigenSTL::vector_Isometry3d& poses, const EigenSTL::vector_Isometry3d& orig, unsigned npoints, unsigned idx)
{
  auto n = npoints / 2;
  poses.insert(poses.end(), orig.begin() + n * idx, orig.begin() + n * (idx + 1));
}








void addVel(trajectory_msgs::JointTrajectory& traj)   //Velocity of the joints
{
  if (traj.points.size() < 3) return;

  auto n_joints = traj.points.front().positions.size();

  for (auto i = 0; i < n_joints; ++i)
  {
    for (auto j = 1; j < traj.points.size() - 1; j++)
    {
      // For each point in a given joint
      double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
      double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
      double v = delta_theta / delta_time;
      traj.points[j].velocities[i] = v;
    } 
  }
}



///See after
void VSLPlanner::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                                      trajectory_msgs::JointTrajectory& out_traj)
{
//  // Fill out information about our trajectory
  out_traj.header.stamp = ros::Time::now();
  out_traj.header.frame_id = config_.world_frame;
  out_traj.joint_names = config_.joint_names;

  descartes_utilities::toRosJointPoints(*robot_model_ptr_, in_traj, 0.4, out_traj.points);
  addVel(out_traj);
}


}