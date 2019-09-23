/* Author: Andre Florindo*/

#include <vsl_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vsl_planner_descartes");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Main program
    vsl_motion_planning::VSLPlanner planner;

    planner.initRos();
    planner.initDescartes();

    EigenSTL::vector_Isometry3d poses;
    planner.getCourse(poses);

    ros::Duration(60).sleep();
    
    std::vector<descartes_core::TrajectoryPtPtr> input_traj;
    planner.generateTrajectory(poses, input_traj);

    std::vector<descartes_core::TrajectoryPtPtr> output_traj;
    planner.planPath(input_traj, output_traj);

    //ros::Duration(5).sleep();

    planner.runPath(output_traj);

    spinner.stop();

    return 0;
}