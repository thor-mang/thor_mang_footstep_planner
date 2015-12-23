#include <thor_mang_footstep_planner/thor_mang_footstep_planner_node.h>

namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

ThorMangFootstepPlannerNode::ThorMangFootstepPlannerNode(ros::NodeHandle& nh)
  : GlobalFootstepPlannerNode(nh)
{
}

ThorMangFootstepPlannerNode::~ThorMangFootstepPlannerNode()
{
}

void ThorMangFootstepPlannerNode::initPlugins(ros::NodeHandle& nh)
{
  GlobalFootstepPlannerNode::initPlugins(nh);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_mang_footstep_planner");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  thor_mang_footstep_planning::ThorMangFootstepPlannerNode thor_mang_planner_node(nh);

  ros::spin();

  return 0;
}
