#include <thor_mang_footstep_planner/thor_mang_step_plan_msg_plugin.h>

namespace thor_mang_footstep_planner
{
using namespace vigir_footstep_planning;

ThorMangStepPlanMsgPlugin::ThorMangStepPlanMsgPlugin()
  : StepPlanMsgPlugin("thor_mang_step_plan_msg_plugin")
{
}

ThorMangStepPlanMsgPlugin::~ThorMangStepPlanMsgPlugin()
{
}



void initStepData(Thor::StepData& step_data)
{
  step_data.PositionData.bMovingFoot = NFootMove;
  step_data.PositionData.dFootHeight = 0.0;
  step_data.PositionData.dZ_Swap_Amplitude = 0.0;

  step_data.PositionData.dShoulderSwingGain = 0.05;
  step_data.PositionData.dElbowSwingGain = 0.1;

  step_data.PositionData.stLeftFootPosition.x = 0.0;
  step_data.PositionData.stLeftFootPosition.y = 0.0;
  step_data.PositionData.stLeftFootPosition.z = 0.0;
  step_data.PositionData.stLeftFootPosition.roll = 0.0;
  step_data.PositionData.stLeftFootPosition.pitch = 0.0;
  step_data.PositionData.stLeftFootPosition.yaw = 0.0;

  step_data.PositionData.stRightFootPosition.x = 0.0;
  step_data.PositionData.stRightFootPosition.y = 0.0;
  step_data.PositionData.stRightFootPosition.z = 0.0;
  step_data.PositionData.stRightFootPosition.roll = 0.0;
  step_data.PositionData.stRightFootPosition.pitch = 0.0;
  step_data.PositionData.stRightFootPosition.yaw = 0.0;

  step_data.PositionData.stBodyPosition.z = BODY_HEIGHT;
  step_data.PositionData.stBodyPosition.roll = 0.0;
  step_data.PositionData.stBodyPosition.pitch = 0.0;
  step_data.PositionData.stBodyPosition.yaw = 0.5*(step_data.PositionData.stRightFootPosition.yaw + step_data.PositionData.stLeftFootPosition.yaw);

  step_data.TimeData.bWalkingState = InWalking;
  step_data.TimeData.dAbsStepTime = 2000;
  step_data.TimeData.dDSPratio = 0.2;
  step_data.TimeData.sigmoid_ratio_x = 1.0;
  step_data.TimeData.sigmoid_ratio_y = 1.0;
  step_data.TimeData.sigmoid_ratio_z = 1.0;
  step_data.TimeData.sigmoid_ratio_roll = 1.0;
  step_data.TimeData.sigmoid_ratio_pitch = 1.0;
  step_data.TimeData.sigmoid_ratio_yaw = 1.0;
  step_data.TimeData.sigmoid_distortion_x = 1.0;
  step_data.TimeData.sigmoid_distortion_y = 1.0;
  step_data.TimeData.sigmoid_distortion_z = 1.0;
  step_data.TimeData.sigmoid_distortion_roll = 1.0;
  step_data.TimeData.sigmoid_distortion_pitch = 1.0;
  step_data.TimeData.sigmoid_distortion_yaw = 1.0;
}

void transformStepPlan(msgs::StepPlan& step_plan, tf::Transform transform)
{
  tf::Pose pose;

  // start pose
  tf::poseMsgToTF(step_plan.start.left.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.start.left.pose);

  tf::poseMsgToTF(step_plan.start.right.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.start.right.pose);

  // goal pose
  tf::poseMsgToTF(step_plan.goal.left.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.goal.left.pose);

  tf::poseMsgToTF(step_plan.goal.right.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.goal.right.pose);

  // entire plan
  for (std::vector<msgs::Step>::iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    msgs::Step& step = *itr;

    tf::poseMsgToTF(step.foot.pose, pose);
    pose = transform * pose;
    tf::poseTFToMsg(pose, step.foot.pose);
  }
}



bool operator<<(Thor::StepData& step_data, const msgs::Step& step)
{
  step_data.PositionData.bMovingFoot = step.foot.foot_index == msgs::Foot::LEFT ? static_cast<int>(LFootMove) : static_cast<int>(RFootMove);
  step_data.PositionData.dFootHeight = step.swing_height * 1000.0;
  step_data.PositionData.dZ_Swap_Amplitude = 10.0;

//  step_data.PositionData.dShoulderSwingGain = 0.05;
//  step_data.PositionData.dElbowSwingGain = 0.1;

  Thor::Pose3D& swing_foot = step.foot.foot_index == msgs::Foot::LEFT ? step_data.PositionData.stLeftFootPosition : step_data.PositionData.stRightFootPosition;
  Thor::Pose3D& stand_foot = step.foot.foot_index == msgs::Foot::LEFT ? step_data.PositionData.stRightFootPosition : step_data.PositionData.stLeftFootPosition;

  double foot_dz = swing_foot.z;

  toThor(step.foot.pose, swing_foot);

  foot_dz = swing_foot.z - foot_dz;

  // no significant change in z
  if (std::abs(foot_dz) < 0.1)
  {
    step_data.PositionData.stBodyPosition.z = BODY_HEIGHT + stand_foot.z;
  }
  // step up
  else if (foot_dz > 0.0)
  {
    // if stand foot has already stepped up then don't lift immediatly body to full high
    if (stand_foot.z > swing_foot.z - foot_dz)
    {
      step_data.PositionData.stBodyPosition.z = BODY_HEIGHT + 0.25*(stand_foot.z+swing_foot.z);
    }
    else
    {
      step_data.PositionData.stBodyPosition.z = BODY_HEIGHT + stand_foot.z;
    }
  }
  // step down
  else
  {
    step_data.PositionData.stBodyPosition.z = BODY_HEIGHT + swing_foot.z;
  }

  step_data.PositionData.dFootHeight = step_data.PositionData.dFootHeight + foot_dz;

//  step_data.PositionData.stBodyPosition.roll = 0.0;
//  step_data.PositionData.stBodyPosition.pitch = 0.0;
  step_data.PositionData.stBodyPosition.yaw = 0.5*(step_data.PositionData.stRightFootPosition.yaw + step_data.PositionData.stLeftFootPosition.yaw);

  step_data.TimeData.bWalkingState = InWalking;
  step_data.TimeData.dAbsStepTime = step.step_duration * 1000.0;
  step_data.TimeData.dDSPratio = 0.2;
//  step_data.TimeData.sigmoid_ratio_x = 1.0;
//  step_data.TimeData.sigmoid_ratio_y = 1.0;
//  step_data.TimeData.sigmoid_ratio_z = 1.0;
//  step_data.TimeData.sigmoid_ratio_roll = 1.0;
//  step_data.TimeData.sigmoid_ratio_pitch = 1.0;
//  step_data.TimeData.sigmoid_ratio_yaw = 1.0;
//  step_data.TimeData.sigmoid_distortion_x = 1.0;
//  step_data.TimeData.sigmoid_distortion_y = 1.0;
//  step_data.TimeData.sigmoid_distortion_z = 1.0;
//  step_data.TimeData.sigmoid_distortion_roll = 1.0;
//  step_data.TimeData.sigmoid_distortion_pitch = 1.0;
//  step_data.TimeData.sigmoid_distortion_yaw = 1.0;

  return true;
}

bool operator<<(std::vector<Thor::StepData>& step_data_list, const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.size() < 2)
    return false;

  msgs::StepPlan _step_plan = step_plan;
  Thor::StepData step_data_curr;

  // generate new plan from scratch
  if (step_data_list.empty())
  {
    // transform plan to be relative to reference start foot pose
    tf::Pose pose;
    tf::poseMsgToTF(step_plan.steps[0].foot.pose, pose);
    tf::Transform transform = pose.inverse();
    transformStepPlan(_step_plan, transform);

    // init with current position
    initStepData(step_data_curr);
    toThor(_step_plan.start.left.pose, step_data_curr.PositionData.stLeftFootPosition);
    toThor(_step_plan.start.right.pose, step_data_curr.PositionData.stRightFootPosition);
    step_data_curr.PositionData.stBodyPosition.z = BODY_HEIGHT + std::min(step_data_curr.PositionData.stLeftFootPosition.z, step_data_curr.PositionData.stRightFootPosition.z);
    step_data_curr.PositionData.stBodyPosition.yaw = 0.5*(step_data_curr.PositionData.stRightFootPosition.yaw + step_data_curr.PositionData.stLeftFootPosition.yaw);
    step_data_curr.TimeData.bWalkingState = InWalkingStarting;

    step_data_list.push_back(step_data_curr);
  }
  // stitch plan
  else
  {
    Thor::StepData& ref_step_data = step_data_list.back();
    const msgs::Step& step = step_plan.steps[0];

    // determine thor's internal start foot position
    Thor::Pose3D ref_thor_foot;
    if (ref_step_data.PositionData.bMovingFoot == NFootMove)
    {
      if (step.foot.foot_index == msgs::Foot::RIGHT)
        ref_thor_foot = ref_step_data.PositionData.stRightFootPosition;
      else
        ref_thor_foot = ref_step_data.PositionData.stLeftFootPosition;
    }
    else if (ref_step_data.PositionData.bMovingFoot == RFootMove && step.foot.foot_index == msgs::Foot::RIGHT)
      ref_thor_foot = ref_step_data.PositionData.stRightFootPosition;
    else if (ref_step_data.PositionData.bMovingFoot == LFootMove && step.foot.foot_index == msgs::Foot::LEFT)
      ref_thor_foot = ref_step_data.PositionData.stLeftFootPosition;
    else
    {
      ROS_ERROR("Start foot in step plan doesn't match the last moving foot of current plan!");
      return false;
    }

    // ref foot pose of thor's internal walking engine
    tf::Pose ref_thor_foot_pose;
    toRos(ref_thor_foot, ref_thor_foot_pose);

    // ref foot pose of plan
    tf::Pose ref_plan_foot_pose;
    tf::poseMsgToTF(step.foot.pose, ref_plan_foot_pose);

    // get transformation 'footstep plan start' -> 'thor start foot'
    tf::Transform transform = ref_thor_foot_pose * ref_plan_foot_pose.inverse();

    // transform plan to be relative to thor's reference foot pose
    transformStepPlan(_step_plan, transform);

    // set correct states
    if (ref_step_data.TimeData.bWalkingState != InWalking)
    {
      ref_step_data.TimeData.bWalkingState = InWalkingStarting;
      ref_step_data.TimeData.dAbsStepTime = 3000;
    }

    step_data_curr = ref_step_data;
  }

  for (std::vector<msgs::Step>::const_iterator itr = _step_plan.steps.begin(); itr != _step_plan.steps.end(); itr++)
  {
    msgs::Step step = *itr;

    // skip inital step of plan
    if (step.step_index == 0)
      continue;

    Thor::StepData step_data_prev = step_data_curr;

    // update with next step
    step_data_curr << *itr;

    // get absolute time index
    step_data_curr.TimeData.dAbsStepTime += step_data_prev.TimeData.dAbsStepTime;

    step_data_list.push_back(step_data_curr);
  }

  // append last step data again as ending
  step_data_curr.PositionData.bMovingFoot = NFootMove;
  step_data_curr.PositionData.dFootHeight = 0.0;
  step_data_curr.PositionData.dZ_Swap_Amplitude = 0.0;
  step_data_curr.TimeData.bWalkingState = InWalkingEnding;
  step_data_curr.TimeData.dAbsStepTime += 2000;
  step_data_list.push_back(step_data_curr);

  return true;
}

bool operator==(const Thor::StepData& lhs, const Thor::StepData& rhs)
{
  return (lhs.PositionData.bMovingFoot == rhs.PositionData.bMovingFoot &&
          lhs.PositionData.dFootHeight == rhs.PositionData.dFootHeight &&
          lhs.PositionData.dZ_Swap_Amplitude == rhs.PositionData.dZ_Swap_Amplitude &&
          lhs.PositionData.dShoulderSwingGain == rhs.PositionData.dShoulderSwingGain &&
          lhs.PositionData.dElbowSwingGain == rhs.PositionData.dElbowSwingGain &&

          std::abs(lhs.PositionData.stLeftFootPosition.x - rhs.PositionData.stLeftFootPosition.x) < 10.0 &&
          std::abs(lhs.PositionData.stLeftFootPosition.y - rhs.PositionData.stLeftFootPosition.y) < 10.0 &&
          std::abs(lhs.PositionData.stLeftFootPosition.z - rhs.PositionData.stLeftFootPosition.z) < 10.0 &&
          std::abs(lhs.PositionData.stLeftFootPosition.roll - rhs.PositionData.stLeftFootPosition.roll) < 0.1 &&
          std::abs(lhs.PositionData.stLeftFootPosition.pitch - rhs.PositionData.stLeftFootPosition.pitch) < 0.1 &&
          std::abs(lhs.PositionData.stLeftFootPosition.yaw - rhs.PositionData.stLeftFootPosition.yaw) < 0.1 &&

          std::abs(lhs.PositionData.stRightFootPosition.x - rhs.PositionData.stRightFootPosition.x) < 10.0 &&
          std::abs(lhs.PositionData.stRightFootPosition.y - rhs.PositionData.stRightFootPosition.y) < 10.0 &&
          std::abs(lhs.PositionData.stRightFootPosition.z - rhs.PositionData.stRightFootPosition.z) < 10.0 &&
          std::abs(lhs.PositionData.stRightFootPosition.roll - rhs.PositionData.stRightFootPosition.roll) < 0.1 &&
          std::abs(lhs.PositionData.stRightFootPosition.pitch - rhs.PositionData.stRightFootPosition.pitch) < 0.1 &&
          std::abs(lhs.PositionData.stRightFootPosition.yaw - rhs.PositionData.stRightFootPosition.yaw) < 0.1 &&

          std::abs(lhs.PositionData.stBodyPosition.z - rhs.PositionData.stBodyPosition.z) < 10.0 &&
          std::abs(lhs.PositionData.stBodyPosition.roll - rhs.PositionData.stBodyPosition.roll) < 0.1 &&
          std::abs(lhs.PositionData.stBodyPosition.pitch - rhs.PositionData.stBodyPosition.pitch) < 0.1 &&
          std::abs(lhs.PositionData.stBodyPosition.yaw - rhs.PositionData.stBodyPosition.yaw) < 0.1 &&

          lhs.TimeData.bWalkingState == rhs.TimeData.bWalkingState &&
          lhs.TimeData.dAbsStepTime == rhs.TimeData.dAbsStepTime &&
          lhs.TimeData.dDSPratio == rhs.TimeData.dDSPratio &&

          lhs.TimeData.sigmoid_ratio_x == rhs.TimeData.sigmoid_ratio_x &&
          lhs.TimeData.sigmoid_ratio_y == rhs.TimeData.sigmoid_ratio_y &&
          lhs.TimeData.sigmoid_ratio_z == rhs.TimeData.sigmoid_ratio_z &&
          lhs.TimeData.sigmoid_ratio_roll == rhs.TimeData.sigmoid_ratio_roll &&
          lhs.TimeData.sigmoid_ratio_pitch == rhs.TimeData.sigmoid_ratio_pitch &&
          lhs.TimeData.sigmoid_ratio_yaw == rhs.TimeData.sigmoid_ratio_yaw &&

          lhs.TimeData.sigmoid_distortion_x == rhs.TimeData.sigmoid_distortion_x &&
          lhs.TimeData.sigmoid_distortion_y == rhs.TimeData.sigmoid_distortion_y &&
          lhs.TimeData.sigmoid_distortion_z == rhs.TimeData.sigmoid_distortion_z &&
          lhs.TimeData.sigmoid_distortion_roll == rhs.TimeData.sigmoid_distortion_roll &&
          lhs.TimeData.sigmoid_distortion_pitch == rhs.TimeData.sigmoid_distortion_pitch &&
          lhs.TimeData.sigmoid_distortion_yaw == rhs.TimeData.sigmoid_distortion_yaw);
}

bool operator!=(const Thor::StepData& lhs, const Thor::StepData& rhs)
{
  return !(lhs == rhs);
}

bool operator==(const std::vector<Thor::StepData>& lhs, const std::vector<Thor::StepData>& rhs)
{
  if (lhs.size() != rhs.size())
    return false;

  // ignore first entry, as start config may differ
  for (size_t i = 1; i < lhs.size(); i++)
  {
    if (lhs[i] != rhs[i])
      return false;
  }

  return true;
}

void toThor(const tf::Pose& pose_in, Thor::Pose3D& pose_out)
{
  geometry_msgs::Pose msg;
  tf::poseTFToMsg(pose_in, msg);
  toThor(msg, pose_out);
}

void toThor(const geometry_msgs::Pose& pose_in, Thor::Pose3D& pose_out)
{
  pose_out.x = pose_in.position.x * 1000.0;
  pose_out.y = pose_in.position.y * 1000.0;
  pose_out.z = pose_in.position.z * 1000.0;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_in.orientation, q);
  tf::Matrix3x3(q).getRPY(pose_out.roll, pose_out.pitch, pose_out.yaw);

  /// HACK
  pose_out.roll = pose_out.pitch = 0.0;
}

void toRos(const Thor::Pose3D& pose_in, tf::Pose& pose_out)
{
  geometry_msgs::Pose msg;
  toRos(pose_in, msg);
  tf::poseMsgToTF(msg, pose_out);
}

void toRos(const Thor::Pose3D& pose_in, geometry_msgs::Pose& pose_out)
{
  pose_out.position.x = pose_in.x / 1000.0;
  pose_out.position.y = pose_in.y / 1000.0;
  pose_out.position.z = pose_in.z / 1000.0;
  pose_out.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_in.roll, pose_in.pitch, pose_in.yaw);
}

std::string toString(const Thor::StepData& step_data)
{
  std::stringstream s;

  s << std::endl;

  s << "[ Step] " << step_data.PositionData.bMovingFoot << " " << step_data.PositionData.dFootHeight << " " << step_data.PositionData.dZ_Swap_Amplitude << " "
    << step_data.PositionData.dShoulderSwingGain << " " << step_data.PositionData.dElbowSwingGain << std::endl;

  s << "[ Left] " << step_data.PositionData.stLeftFootPosition.x << " " << step_data.PositionData.stLeftFootPosition.y << " " <<step_data.PositionData.stLeftFootPosition.z << " "
    << step_data.PositionData.stLeftFootPosition.roll << " " <<step_data.PositionData.stLeftFootPosition.pitch << " " << step_data.PositionData.stLeftFootPosition.yaw << std::endl;

  s << "[Right] " << step_data.PositionData.stRightFootPosition.x << " " << step_data.PositionData.stRightFootPosition.y << " " <<step_data.PositionData.stRightFootPosition.z << " "
    << step_data.PositionData.stRightFootPosition.roll << " " <<step_data.PositionData.stRightFootPosition.pitch << " " << step_data.PositionData.stRightFootPosition.yaw << std::endl;

  s << "[ Body] " << step_data.PositionData.stBodyPosition.z << " " << step_data.PositionData.stBodyPosition.roll << " " << step_data.PositionData.stBodyPosition.pitch << " " << step_data.PositionData.stBodyPosition.yaw << std::endl;

  s << "[ Time]" << step_data.TimeData.bWalkingState << " " << step_data.TimeData.dAbsStepTime << " " << step_data.TimeData.dDSPratio << std::endl;

  s << "[Ratio]" << step_data.TimeData.sigmoid_ratio_x << " " << step_data.TimeData.sigmoid_ratio_y << " " << step_data.TimeData.sigmoid_ratio_z << " " << step_data.TimeData.sigmoid_ratio_roll << " " << step_data.TimeData.sigmoid_ratio_pitch << " " << step_data.TimeData.sigmoid_ratio_yaw << std::endl;

  s << "[Sigmo]" << step_data.TimeData.sigmoid_distortion_x << " " << step_data.TimeData.sigmoid_distortion_y << " " << step_data.TimeData.sigmoid_distortion_z << " " << step_data.TimeData.sigmoid_distortion_roll << " " << step_data.TimeData.sigmoid_distortion_pitch << " " << step_data.TimeData.sigmoid_distortion_yaw << std::endl;

  return s.str();
}
}
