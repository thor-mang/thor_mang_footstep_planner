#include <thor_mang_footstep_planner/thor_mang_footstep_planner_node.h>

namespace thor_mang_footstep_planner
{
using namespace vigir_footstep_planning;

AtlasFootstepPlannerNode::AtlasFootstepPlannerNode()
  : GlobalFootstepPlannerNode()
{
}

AtlasFootstepPlannerNode::~AtlasFootstepPlannerNode()
{
}

void AtlasFootstepPlannerNode::initPlugins(ros::NodeHandle& nh)
{
  GlobalFootstepPlannerNode::initPlugins(nh);
  PluginManager::addPlugin<ThorMangStepPlanMsgPlugin>();
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_mang_footstep_planner");

  ros::NodeHandle nh;
  thor_mang_footstep_planner::AtlasFootstepPlannerNode thorMangPlannerNode;
  thorMangPlannerNode.initPlugins(nh);
  thorMangPlannerNode.init(nh);

  ros::spin();

  return 0;
}
