#include <thor_mang_footstep_planner/thor_mang_reachability.h>

namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

ThorMangReachability::ThorMangReachability(const vigir_generic_params::ParameterSet& params)
  : ReachabilityPlugin("thor_mang_reachability", params)
{
}

ThorMangReachability::ThorMangReachability()
  : ReachabilityPlugin("thor_mang_reachability")
{
}

void ThorMangReachability::loadParams(const vigir_generic_params::ParameterSet& /*params*/)
{
  ros::NodeHandle nh;
  nh.getParam("foot/separation", foot_seperation);
}

bool ThorMangReachability::isReachable(const State& /*current*/, const State& /*next*/) const
{
  return true;
}

bool ThorMangReachability::isReachable(const State& left_foot, const State& right_foot, const State& swing_foot) const
{
  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;

  msgs::Foot _stand_foot;
  stand_foot.getFoot(_stand_foot);

  msgs::Foot _swing_foot;
  swing_foot.getFoot(_swing_foot);

  // reconstruct delta step
  geometry_msgs::Pose dstep;
  getDeltaStep(_stand_foot, _swing_foot, dstep);
  double dstep_yaw = tf::getYaw(dstep.orientation);

  // forbid toe in
  if (dstep_yaw < -0.001)
    return false;

  msgs::Foot _swing_foot_before;
  swing_foot_before.getFoot(_swing_foot_before);

  // reconstruct delta for swing foot
  geometry_msgs::Pose dswing;
  getDeltaStep(_swing_foot_before, _swing_foot, dswing);
  double dswing_yaw = tf::getYaw(dswing.orientation);

  // restrict to RTR movement
  if (std::abs(dswing.position.x) > 0.01 && std::abs(dswing.position.y) > 0.01)
    return false;

  if (std::abs(dswing.position.x) > 0.05 && std::abs(dswing_yaw) > 0.01)
    return false;

  if (std::abs(dswing.position.y) > 0.01 && std::abs(dswing_yaw) > 0.01)
    return false;

  return true;
}
}
