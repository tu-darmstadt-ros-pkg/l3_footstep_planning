#include <l3_footstep_planning_vis_tools/footstep_planning_vis_node.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_vis/visualization.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_tools/foot_pose_transformer.h>

#include <l3_footstep_planning_vis_tools/visualization.h>

namespace l3_footstep_planning
{
FootstepPlanningVisNode::FootstepPlanningVisNode(ros::NodeHandle& nh)
  : vis_plugins_("PlanningVisPlugins")
{
  ros::NodeHandle pnh("~");

  // initialize plugin manager
  vigir_pluginlib::PluginManager::initialize(nh);

  // initialize robot model
  RobotModel::initialize(nh);

  // initialize foot pose transformer
  if (!RobotModel::description())
  {
    ROS_ERROR("[FootstepPlanningVisNode] No robot description available!");
    return;
  }
  FootPoseTransformer::initialize(RobotModel::description());

  // register class loader
  vigir_pluginlib::PluginManager::addPluginClassLoader<PlanningVisPlugin>("l3_footstep_planning_vis_tools", "l3_footstep_planning::PlanningVisPlugin");

  // load plugin set
  std::string plugin_set_name = pnh.param("plugin_set", std::string("vis"));
  if (loadPluginSet(plugin_set_name))
    vis_plugins_.loadPlugins();

  // subscribe topics
  load_plugin_set_sub_ = nh.subscribe<std_msgs::String>("vis/load_plugin_set", 1, &FootstepPlanningVisNode::loadPluginSetCallBack, this);
  step_plan_request_vis_sub_ = nh.subscribe<msgs::StepPlanRequest>("vis/step_plan_request", 1, &FootstepPlanningVisNode::stepPlanRequestVisCallback, this);
  step_plan_vis_sub_ = nh.subscribe<msgs::StepPlan>("vis/step_plan", 1, &FootstepPlanningVisNode::stepPlanVisCallback, this);
  planning_feedback_sub_ = nh.subscribe<msgs::PlanningFeedback>("planning_feedback", 1, &FootstepPlanningVisNode::planningFeedbackCallback, this);

  // publish topics
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vis/markers", 1, true);

  // start auto publisher
  double rate = pnh.param("auto_pub_rate", 10.0);
  if (rate > 0.0)
    auto_publisher_ = nh.createTimer(ros::Rate(rate).expectedCycleTime(), [this](const ros::TimerEvent&) { this->publishMarkerArray(); });
}

bool FootstepPlanningVisNode::loadPluginSet(const std::string& name)
{
  if (!vigir_pluginlib::PluginManager::loadPluginSet(name))
  {
    ROS_ERROR("[FootstepPlanningVisNode] Failed to load vis plugins!");
    return false;
  }
  return true;
}

void FootstepPlanningVisNode::stepPlanRequestVisCallback(const msgs::StepPlanRequestConstPtr& step_plan_request)
{
  // clear old vis
  clearVisualization();

  // transform to planner frame
  msgs::StepPlanRequest step_plan_request_t = *step_plan_request;
  FootPoseTransformer::transformToPlannerFrame(step_plan_request_t.start_footholds);
  FootPoseTransformer::transformToPlannerFrame(step_plan_request_t.goal_footholds);

  vis_plugins_.call([step_plan_request_t](PlanningVisPlugin::Ptr p) { p->visualize(step_plan_request_t); });
  vis_plugins_.call([this](PlanningVisPlugin::Ptr p) { this->updateMarker(p->getMarkerArray()); });
  vis_plugins_.call([](PlanningVisPlugin::Ptr p) { p->trigger(); });

  publishMarkerArray();
}

void FootstepPlanningVisNode::stepPlanVisCallback(const msgs::StepPlanConstPtr& step_plan)
{
  // clear old vis
  clearVisualization();

  // transform to planner frame
  msgs::StepPlan step_plan_t = *step_plan;
  FootPoseTransformer::transformToPlannerFrame(step_plan_t);
  StepPlan step_plan_l3(step_plan_t);

  vis_plugins_.call([step_plan_l3](PlanningVisPlugin::Ptr p) { p->visualize(step_plan_l3); });
  vis_plugins_.call([this](PlanningVisPlugin::Ptr p) { this->updateMarker(p->getMarkerArray()); });
  vis_plugins_.call([](PlanningVisPlugin::Ptr p) { p->trigger(); });

  publishMarkerArray();
}

void FootstepPlanningVisNode::planningFeedbackCallback(const msgs::PlanningFeedbackConstPtr& planning_feedback)
{
  // transform to planner frame
  msgs::PlanningFeedback planning_feedback_t = *planning_feedback;
  FootPoseTransformer::transformToPlannerFrame(planning_feedback_t.visited_steps);
  FootPoseTransformer::transformToPlannerFrame(planning_feedback_t.last_visited_state);
  FootPoseTransformer::transformToPlannerFrame(planning_feedback_t.last_visited_step);
  FootPoseTransformer::transformToPlannerFrame(planning_feedback_t.current_step_plan);

  vis_plugins_.call([planning_feedback_t](PlanningVisPlugin::Ptr p) { p->visualize(planning_feedback_t); });
  vis_plugins_.call([this](PlanningVisPlugin::Ptr p) { this->updateMarker(p->getMarkerArray()); });
  vis_plugins_.call([](PlanningVisPlugin::Ptr p) { p->trigger(); });

  publishMarkerArray();
}

void FootstepPlanningVisNode::clearVisualization()
{
  vis_plugins_.call([](PlanningVisPlugin::Ptr p) { p->clear(); });
  vis_plugins_.call([](PlanningVisPlugin::Ptr p) { p->trigger(); });

  markers_.clear();
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(createResetMarker());
  marker_pub_.publish(markers);
}

void FootstepPlanningVisNode::updateMarker(const visualization_msgs::MarkerArray& markers)
{
  std::map<std::string, visualization_msgs::MarkerArray> update;
  std::set<std::string> cleared_ns;

  // sort markers by namespace
  for (const visualization_msgs::Marker& marker : markers.markers)
  {
    update[marker.ns].markers.push_back(marker);
    if (marker.action == visualization_msgs::Marker::DELETEALL)
      cleared_ns.insert(marker.ns);
  }

  // handle DELETEALL markers
  for (const std::string& ns : cleared_ns)
    update[ns].markers.clear();

  // merge into marker map
  for (const std::pair<std::string, visualization_msgs::MarkerArray>& p : update)
    l3::updateMarkerArray(markers_[p.first], p.second);
}

void FootstepPlanningVisNode::publishMarkerArray()
{
  visualization_msgs::MarkerArray all_markers;

  for (std::pair<const std::string, visualization_msgs::MarkerArray> p : markers_)
  {
    visualization_msgs::MarkerArray& markers = p.second;

    // copy markers to publish
    all_markers.markers.insert(all_markers.markers.end(), markers.markers.begin(), markers.markers.end());

    // remove all markers which are tagged as deleted
    l3::removeDeletedMarkers(markers);
  }

  marker_pub_.publish(all_markers);
}
}  // namespace l3_footstep_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_planning_vis_node");

  ros::NodeHandle nh;

  l3_footstep_planning::FootstepPlanningVisNode vis_node(nh);
  ros::spin();

  return 0;
}
