/* Author: Andre Florindo*/

#include <vsl_planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Main program
    vsl_motion_planning::VSLPlanner planner;

    planner.initRos();
    planner.initDescartes();

    CourseStruct course;
    EigenSTL::vector_Isometry3d poses;
    planner.readFileContent(course, poses);

    std::vector<descartes_core::TrajectoryPtPtr> input_traj;
    planner.generateTrajectory(poses, input_traj);

    std::vector<descartes_core::TrajectoryPtPtr> output_traj;
    planner.planPath(input_traj, output_traj);

    //planner.runPath(output_traj);

    spinner.stop();

    return 0;
}