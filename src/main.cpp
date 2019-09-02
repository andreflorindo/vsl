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
    EigenSTL::vector_Isometry3d poses;
    PosesPtr = &poses;
    planner.readFileContent(CoursePtr, PosesPtr);

    poses.size()

    for (int i = 0; i < poses.size(); i++)
    {
        if (i % 2 == 0)
        {
            std::cout << i << "   " << poses[i](1, 1) << std::endl;
        }
    }

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