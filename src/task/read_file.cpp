/* Author: Andre Florindo*/

// Add vector<CourseStruct> in case there are more courses
// If z is not given in the file, maybe add a collumn of zeros

#include <vsl_application.h>

bool getFileContent(CourseStruct *&course)
{
    //     https://stackoverflow.com/questions/46663046/save-read-double-vector-from-file-c                    //<-------------  Other way

    std::ifstream infile{"/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/simplePath.txt", std::ios::in};

    if (!infile.good())
    {
        ROS_ERROR_NAMED("vsl", "Path as not able to be found");
        return false;
    }
    
    std::istream_iterator<double> infile_begin{infile};
    std::istream_iterator<double> eof{};
    std::vector<double> file_nums{infile_begin, eof};
    infile.close();

    int nx = 0;
    int ny = 0;

    course->x.reserve(file_nums.size() / 3);
    course->y.reserve(file_nums.size() / 3);
    course->z.reserve(file_nums.size() / 3);

    for (int i = 0; i < file_nums.size(); i++)
    {
        if (i == nx * 3)
        {
            course->x.emplace_back(file_nums[i]);
            nx++;
        }
        else if (i == 1 + ny * 3)
        {
            course->y.emplace_back(file_nums[i]);
            ny++;       
        }
        else
            course->z.emplace_back(file_nums[i]);
        
    }
    ROS_INFO("File found, reading complete");

    return true;

};

// bool DemoApplication::createLemniscateCurve(//double foci_distance, double sphere_radius,
//                                  // int num_points, int num_lemniscates,
//                                   const Eigen::Vector3d& sphere_center,
//                                   EigenSTL::vector_Isometry3d& poses)

// {
//   std::ifstream infile{"/home/andreflorindo/workspaces/vsl_msc_project_ws/src/vsl_core/examples/simplePath.txt", std::ios::in};

//   if (!infile.good())
//   {
//       ROS_ERROR_NAMED("vsl", "Path as not able to be found");
//       return false;
//   }
    
//   std::istream_iterator<double> infile_begin{infile};
//   std::istream_iterator<double> eof{};
//   std::vector<double> file_nums{infile_begin, eof};
//   std::vector<double> x, y, z;
//   infile.close();

//   int nx = 0;
//   int ny = 0;
//   int npoints = file_nums.size()/3;
//   //int nlemns = num_lemniscates;

//   x.reserve(npoints);
//   y.reserve(npoints);
//   z.reserve(npoints);

//   for (int i = 0; i < file_nums.size(); i++)
//   {
//       if (i == nx * 3)
//       {
//           x.emplace_back(file_nums[i]);
//           nx++;
//       }
//       else if (i == 1 + ny * 3)
//       {
//           y.emplace_back(file_nums[i]);
//           ny++;       
//       }
//       else
//           z.emplace_back(file_nums[i]);
        
//   }
  
//   Eigen::Vector3d offset(sphere_center[0],sphere_center[1],sphere_center[2]);
//   Eigen::Vector3d unit_z,unit_y,unit_x;
//   Eigen::Isometry3d pose;

//   poses.clear();
//   //poses.reserve(nlemns*npoints);
//   poses.reserve(npoints);

//   //for(unsigned int j = 0; j < nlemns;j++)
//   //{
//     for(unsigned int i = 0 ; i < npoints;i++)
//     {
//       // determining orientation
//       unit_z <<-x[i], -y[i] , -z[i];
//       unit_z.normalize();

//       unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
//       unit_y = (unit_z .cross(unit_x)).normalized();

//       Eigen::Isometry3d rot;
//       rot.matrix() << unit_x(0),unit_y(0),unit_z(0),0
//          ,unit_x(1),unit_y(1),unit_z(1),0
//          ,unit_x(2),unit_y(2),unit_z(2),0
//          ,0,0,0,1;



//       pose = Eigen::Translation3d(offset(0) + x[i],
//                                   offset(1) + y[i],
//                                   offset(2) + z[i]) * rot;

//       poses.push_back(pose);
//     }
//     //std::reverse(poses.end() - npoints / 2, poses.end());
//   //}
//   return true;
// }
