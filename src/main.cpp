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

    //CourseStruct *course = new CourseStruct;
    std::vector<EigenSTL::vector_Isometry3d> *course = new std::vector<EigenSTL::vector_Isometry3d>
    EigenSTL::vector_Isometry3d *poses = new EigenSTL::vector_Isometry3d;

    planner.ReadFileContent(course, poses);

    std::vector<descartes_core::TrajectoryPtPtr> traj;

    planner.generateTrajectory(poses, traj);


// std::vector<descartes_core::TrajectoryPtPtr> output_path;
  // planner.planPath(traj,output_path);

  // // running robot path
  // planner.runPath(output_path);




    //const int result = ReadFileContent(course);

    /*for (int i = 0; i < course->x.size(); i++)
    {
        std::cout << course->y[i]<< std::endl;
    }
    */
    
    spinner.stop();

    return 0;
}