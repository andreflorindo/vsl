#include <kukakr210r2700extra_descartes/kukakr210r2700extra_robot_model.h>

namespace kukakr210r2700extra_descartes
{

KukaRobotModel::KukaRobotModel(){}
KukaRobotModel::~KukaRobotModel()(){}

bool KukaRobotModel::initialize(const std::string &robot_description, const std::string& group_name,
                          const std::string& world_frame,const std::string& tcp_frame)
{
  check_collisions_ = true;
  if(!MoveitStateAdapter::initialize(robot_description,group_name,world_frame,tcp_frame))
  {
    ROS_ERROR_STREAM("MoveitStateAdapter from within Kuka robot model failed to initialize");
    return false;
  }


  if(!Kuka_kinematics::KukaKinematicsPlugin::initialize(robot_description, group_name, BASE_LINK,TIP_LINK, 0.001))
  {
    ROS_ERROR_STREAM("KukaKinematicsPlugin from within Kuka robot model failed to initialize");
    return false;
  }

  // initialize world transformations
  if(tcp_frame != getTipFrame())
  {
    tool_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tcp_frame).inverse()*
                                         robot_state_->getFrameTransform(getTipFrame()));
  }
  if(world_frame != getBaseFrame())
  {
    world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(getBaseFrame()));
  }

  ROS_WARN_STREAM("Kuka Descartes Robot Model initialized");

  return true;
}


bool KukaRobotModel::getAllIK(const Eigen::Isometry3d &pose, std::vector<std::vector<double> > &joint_poses) const
{
  using namespace kuka_kinematics;

  bool rtn = false;
  KDL::Frame frame;
  Eigen::Isometry3d tool_pose = world_to_base_.frame_inv* pose* tool_to_tip_.frame;
  tf::transformEigenToKDL(tool_pose, frame);
  joint_poses.clear();

  KDL::JntArray jnt_seed_state(dimension_);
   for(int i=0; i<dimension_; i++)
     jnt_seed_state(i) = 0.0f;


   KDL::ChainFkSolverPos_recursive fk_solver_base(kdl_base_chain_);
   KDL::ChainFkSolverPos_recursive fk_solver_tip(kdl_tip_chain_);

   KDL::JntArray jnt_pos_test(jnt_seed_state);
   KDL::JntArray jnt_pos_base(kuka_joint_inds_start_);
   KDL::JntArray jnt_pos_tip(dimension_ - 6 - kuka_joint_inds_start_);
   KDL::Frame pose_base, pose_tip;

  double ik_pose[4][4];
  double q_ik_sols[8][6]; // maximum of 8 IK solutions
  uint16_t num_sols;
  frame.Make4x4((double*) ik_pose);

  num_sols = inverse((double*) ik_pose, (double*) q_ik_sols,jnt_pos_test(kuka_joint_inds_start_+5));

  ROS_DEBUG_STREAM("kuka custom ikfast found "<<num_sols<<" solutions");
  uint16_t num_valid_sols;
  for(uint16_t i=0; i<num_sols; i++)
  {
   bool valid = true;
   std::vector< double > valid_solution;
   valid_solution.assign(6,0.0);

   for(uint16_t j=0; j<6; j++)
   {
     if((q_ik_sols[i][j] <= ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j] >= ik_chain_info_.limits[j].min_position))
     {
       valid_solution[j] = q_ik_sols[i][j];
       valid = true;
       continue;
     }
     else if ((q_ik_sols[i][j] > ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j]-2*M_PI > ik_chain_info_.limits[j].min_position))
     {
       valid_solution[j] = q_ik_sols[i][j]-2*M_PI;
       valid = true;
       continue;
     }
     else if ((q_ik_sols[i][j] < ik_chain_info_.limits[j].min_position) && (q_ik_sols[i][j]+2*M_PI < ik_chain_info_.limits[j].max_position))
     {
       valid_solution[j] = q_ik_sols[i][j]+2*M_PI;
       valid = true;
       continue;
     }
     else
     {
       valid = false;
       break;
     }
   }

   if(valid && isValid(valid_solution))
   {
     joint_poses.push_back(valid_solution);
   }
  }

  if(joint_poses.empty())
  {
    ROS_WARN_STREAM("GetAllIK has not solutions");
    rtn = false;
  }
  else
  {
    rtn = true;
  }

  return rtn;
}

static double jointCost(const std::vector<double>& source, const std::vector<double>& target)
{
  ROS_ASSERT(source.size() == target.size());
  double cost = 0.0;
  for (size_t i = 0; i < source.size(); ++i)
  {
    cost += std::abs(target[i] - source[i]);
  }
  return cost;
} 

bool KukaRobotModel::getIK(const Eigen::Isometry3d &pose, const std::vector<double> &seed_state,
                          std::vector<double> &joint_pose) const
{
  // forward to getAllIK
  std::vector<std::vector<double> > joint_poses;
  if (!getAllIK(pose, joint_poses))
  {
    return false;
  }
  // Find closest solution in joint-space
  // getAllIK assures at least 1 solution is available
  size_t best_idx = 0;
  double best_cost = jointCost(seed_state, joint_poses[0]);
  for (size_t i = 1; i < joint_poses.size(); ++i)
  {
    double cost = jointCost(seed_state, joint_poses[i]);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_idx = i;
    }
  }
  
  joint_pose = joint_poses[best_idx];
  return true;
}

} 
