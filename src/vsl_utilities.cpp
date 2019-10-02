#include <vsl_planner.h>
#include <descartes_utilities/ros_conversions.h>

namespace vsl_motion_planning
{

void swap_segments(EigenSTL::vector_Isometry3d &poses, unsigned npoints, unsigned idx1, unsigned idx2)
{
  auto n = npoints / 2;
  std::swap_ranges(poses.begin() + n * idx1, poses.begin() + n * (idx1 + 1),
                   poses.begin() + n * idx2);
}

void insert_segment(EigenSTL::vector_Isometry3d &poses, const EigenSTL::vector_Isometry3d &orig, unsigned npoints, unsigned idx)
{
  auto n = npoints / 2;
  poses.insert(poses.end(), orig.begin() + n * idx, orig.begin() + n * (idx + 1));
}

} // namespace vsl_motion_planning