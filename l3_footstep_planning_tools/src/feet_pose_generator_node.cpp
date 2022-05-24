#include <l3_footstep_planning_tools/feet_pose_generator_node.h>

namespace l3_footstep_planning
{
FeetPoseGeneratorNode::FeetPoseGeneratorNode(ros::NodeHandle& nh)
  : feet_pose_generator_(nh)
{
  // subscribe topics
  robot_pose_sub_ = nh.subscribe("/robot_pose", 1, &FeetPoseGenerator::setRobotPose, &feet_pose_generator_);
  robot_pose_with_cov_sub_ = nh.subscribe("/initialpose", 1, &FeetPoseGenerator::setRobotPoseWithCovariance, &feet_pose_generator_);
  terrain_model_sub_ = nh.subscribe("/terrain_model", 1, &FeetPoseGenerator::setTerrainModel, &feet_pose_generator_);

  // start own services
  generate_feet_pose_srv_ = nh.advertiseService("generate_feet_pose", &FeetPoseGeneratorNode::generateFeetPoseService, this);

  // clang-format off
  // init action servers
  generate_feet_pose_as_ = SimpleActionServer<msgs::GenerateFeetPoseAction>::create(nh, "generate_feet_pose", true, boost::bind(&FeetPoseGeneratorNode::generateFeetPoseAction, this, boost::ref(generate_feet_pose_as_)));
  // clang-format off
}

FeetPoseGeneratorNode::~FeetPoseGeneratorNode() {}

// --- service calls ---

bool FeetPoseGeneratorNode::generateFeetPoseService(msgs::GenerateFeetPoseService::Request& req, msgs::GenerateFeetPoseService::Response& resp)
{
  resp.status = feet_pose_generator_.generateFeetPose(req.request, resp.feet);
  return true;  // return always true so the message is returned
}

//--- action server calls ---

void FeetPoseGeneratorNode::generateFeetPoseAction(SimpleActionServer<msgs::GenerateFeetPoseAction>::Ptr& as)
{
  const msgs::GenerateFeetPoseGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GenerateFeetPoseResult result;
  result.status = feet_pose_generator_.generateFeetPose(goal->request, result.feet);

  actionServerFinished(*as, result);
}
}  // namespace l3_footstep_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feet_pose_generator");
  ros::NodeHandle nh;
  l3_footstep_planning::FeetPoseGeneratorNode generator(nh);
  ros::spin();

  return 0;
}
