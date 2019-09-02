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

    CourseStruct *CoursePtr = new CourseStruct;
    EigenSTL::vector_Isometry3d *PosesPtr = new EigenSTL::vector_Isometry3d;
    planner.readFileContent(CoursePtr, PosesPtr);

    std::vector<descartes_core::TrajectoryPtPtr> TrajPtr;
    planner.generateTrajectory(PosesPtr, TrajPtr);

    std::vector<descartes_core::TrajectoryPtPtr> OutTrajPtr;
    planner.planPath(TrajPtr, OutTrajPtr);

    //planner.runPath(OutTrajPtr);




    //const int result = ReadFileContent(course);

    /*for (int i = 0; i < course->x.size(); i++)
    {
        std::cout << course->y[i]<< std::endl;
    }
    */
    
    spinner.stop();

    return 0;
}