#include "slaw_dwa_local_planner/path_alignment_cost_function.h"

#include <math.h>

namespace slaw_dwa_local_planner
{
double PathAlignmentCostFunction::scoreTrajectory(Trajectory &traj)
{
    return fabs(traj.yv_ * getScale());
}
}
