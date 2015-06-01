#include <thor_mang_footstep_planner/thor_mang_heuristic.h>

namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

ThorMangHeuristic::ThorMangHeuristic(const ParameterSet& params)
: HeuristicPlugin("thor_mang_heuristic", params)
{}

ThorMangHeuristic::ThorMangHeuristic()
: HeuristicPlugin("thor_mang_heuristic")
{}

void ThorMangHeuristic::loadParams(const ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("thor_mang_heuristic/step_cost", step_cost, 0.1);
  params.getParam("thor_mang_heuristic/diff_angle_cost", diff_angle_cost, 0.1);
  params.getParam("max_step_dist/x", max_step_dist_x_inv);
  max_step_dist_x_inv = 1.0/max_step_dist_x_inv;
  params.getParam("max_step_dist/y", max_step_dist_y_inv);
  max_step_dist_y_inv = 1.0/max_step_dist_y_inv;
}

double ThorMangHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // determine swing dist
  tf::Transform dstep;
  getDeltaStep(from.getPose(), to.getPose(), dstep);
  double expected_steps_x = std::abs(dstep.getOrigin().x()) * max_step_dist_x_inv;
  double expected_steps_y = std::abs(dstep.getOrigin().y()) * max_step_dist_y_inv;

  if (expected_steps_x > 0.1 && expected_steps_y > 0.1) // rotation suggested
    return expected_steps_y * step_cost;
    //return expected_steps_y/expected_steps_x * step_cost * gain;
  else if (expected_steps_x > 0.1) // transversal
    return expected_steps_x * step_cost;
  else if (expected_steps_y > 0.1) // sidewards
    return expected_steps_y * step_cost;
  else // rotate towards goal orientation
  {
    //ROS_INFO("%f", tf::getYaw(dstep.getRotation()));
    return std::abs(tf::getYaw(dstep.getRotation())) * diff_angle_cost;
  }

  return 0.0;
}
}
