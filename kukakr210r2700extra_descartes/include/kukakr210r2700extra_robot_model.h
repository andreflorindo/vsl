#ifndef KUKAKR210R2700EXTRA_DESCARTES_KUKAKR210R2700EXTRA_ROBOT_MODEL_H_
#ifndef KUKAKR210R2700EXTRA_DESCARTES_KUKAKR210R2700EXTRA_ROBOT_MODEL_H_

#include <descartes_moveit/moveit_state_adapter.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <kukakr210r2700extra_descartes/kuka_moveit_plugin.h>
#include <kukakr210r2700extra_descartes/kuka_kin.h>


namespace kukakr210r2700extra_descartes
{

const std::string BASE_LINK = "base_link";
const std::string TIP_LINK = "ee_link";

class KukaRobotModel: public descartes_moveit::MoveitStateAdapter, public kuka_kinematics::KukaKinematicsPlugin
{
public:
  KukaRobotModel();
  virtual ~KukaRobotModel();

  virtual bool initialize(const std::string &robot_description, const std::string& group_name,
                            const std::string& world_frame,const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getIK(const Eigen::Isometry3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  descartes_core::Frame world_to_base_;// world to arm base
  descartes_core::Frame tool_to_tip_; // from arm tool to robot tool

};

} 

#endif
