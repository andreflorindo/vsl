  
  static const std::string PLANNING_GROUP = "Manipulator"; // Defining the planning group name
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // Setting up the move group interface based on name
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // Defening the planning scene interface
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); // Raw pointer to the planning group?

  // Vizualisation
  namespace rvt = rviz_visual_tools; 
  moveit_visual_tools::MoveItVisualTools visual_tools("kr210_base_link"); 
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 2.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ros::Duration(5.0).sleep();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
