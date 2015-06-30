#include <thor_mang_footstep_planner/thor_mang_footstep_planner_node.h>

#include <thor_mang_footstep_planning_msgs/thor_mang_step_plan_msg_plugin.h>
#include <thor_mang_footstep_planner/thor_mang_reachability.h>
#include <thor_mang_footstep_planner/thor_mang_heuristic.h>

namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

ThorMangFootstepPlannerNode::ThorMangFootstepPlannerNode()
  : GlobalFootstepPlannerNode()
{
}

ThorMangFootstepPlannerNode::~ThorMangFootstepPlannerNode()
{
}

void ThorMangFootstepPlannerNode::initPlugins(ros::NodeHandle& nh)
{
  GlobalFootstepPlannerNode::initPlugins(nh);

  vigir_pluginlib::PluginManager::removePluginsByType<StepCostHeuristic>();

  vigir_pluginlib::PluginManager::addPlugin<ThorMangStepPlanMsgPlugin>();
  vigir_pluginlib::PluginManager::addPlugin<ThorMangReachability>();
  vigir_pluginlib::PluginManager::addPlugin<ThorMangHeuristic>();
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_mang_footstep_planner");

  ros::NodeHandle nh;
  thor_mang_footstep_planning::ThorMangFootstepPlannerNode thorMangPlannerNode;
  thorMangPlannerNode.initPlugins(nh);
  thorMangPlannerNode.init(nh);

  ros::spin();

  return 0;
}
