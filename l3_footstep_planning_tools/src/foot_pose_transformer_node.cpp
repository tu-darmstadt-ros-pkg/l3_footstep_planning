#include <l3_footstep_planning_tools/foot_pose_transformer_node.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

namespace l3_footstep_planning
{
FootPoseTransformerNode::FootPoseTransformerNode(ros::NodeHandle& nh)
{
  // initialize transformer
  if (!FootPoseTransformer::initialize(nh))
  {
    ROS_ERROR("[FootPoseTransformerNode] Initialization of FootPoseTransformer failed!");
    return;
  }

  // start own services
  transform_foot_pose_srv_ = nh.advertiseService("transform_foot_pose", &FootPoseTransformerNode::transformFootPoseService, this);
  transform_feet_poses_srv_ = nh.advertiseService("transform_feet_poses", &FootPoseTransformerNode::transformFeetPosesService, this);
  transform_step_plan_srv_ = nh.advertiseService("transform_step_plan", &FootPoseTransformerNode::transformStepPlanService, this);
}

FootPoseTransformerNode::~FootPoseTransformerNode() {}

bool FootPoseTransformerNode::transformFootPoseService(msgs::TransformFootPoseService::Request& req, msgs::TransformFootPoseService::Response& resp)
{
  resp.foot = req.foot;
  resp.status = FootPoseTransformer::transform(resp.foot, req.target_frame.data);
  return true;  // return always true so the message is returned
}

bool FootPoseTransformerNode::transformFeetPosesService(msgs::TransformFeetPosesService::Request& req, msgs::TransformFeetPosesService::Response& resp)
{
  resp.feet = req.feet;
  resp.status = FootPoseTransformer::transform(resp.feet, req.target_frame.data);
  return true;  // return always true so the message is returned
}

bool FootPoseTransformerNode::transformStepPlanService(msgs::TransformStepPlanService::Request& req, msgs::TransformStepPlanService::Response& resp)
{
  resp.step_plan = req.step_plan;
  resp.status = FootPoseTransformer::transform(resp.step_plan, req.target_frame.data);
  return true;  // return always true so the message is returned
}
}  // namespace l3_footstep_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foot_pose_transformer_node");
  ros::NodeHandle nh;
  l3_footstep_planning::FootPoseTransformerNode foot_pose_transformer_node(nh);
  ros::spin();

  return 0;
}
