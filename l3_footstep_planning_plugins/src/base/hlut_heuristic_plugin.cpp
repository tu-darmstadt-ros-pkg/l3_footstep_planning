#include <l3_footstep_planning_plugins/base/hlut_heuristic_plugin.h>

namespace l3_footstep_planning
{
HLUTHeuristicPlugin::HLUTHeuristicPlugin(const std::string& name)
  : HeuristicPlugin(name)
{}

bool HLUTHeuristicPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  // get the parameters
  if (hasParam("resolution"))
    resolution_ = DiscreteResolution(getSubset("resolution"));
  else
    resolution_ = DiscreteResolution(params.getSubset("resolution"));
  getParam("size_x", size_x_, 10);
  getParam("size_y", size_y_, 10);
  getParam("check_foothold_accessibility", check_foothold_accessibility_, false, true);
  getParam("check_floating_base_accessibility", check_floating_base_accessibility_, false, true);
  getParam("visualize", visualize_, false);
  getParam("vis_frame_id", vis_frame_id_, std::string("map"), !visualize_);
  getParam("vis_topic", vis_topic_, std::string(getName() + "_values"), !visualize_);
  getParam("vis_layer", vis_layer_, std::string(getName() + "_values"), !visualize_);

  if (resolution_.numAngleBins() <= 0)
  {
    ROS_ERROR("[%s]: num_angle_bins_ must be greater than 0", getName().c_str());
    return false;
  }

  // calculate angle_bins_
  for (int i = 0; i < resolution_.numAngleBins(); i++)
    angle_bins_.push_back(normalizeAngle(i * resolution_.resolution().angle - M_PI));

  return true;
}

bool HLUTHeuristicPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(params))
    return false;

  // initialize HLUT publisher if visualization is enabled
  if (visualize_)
    hlut_pub_ = nh_.advertise<grid_map_msgs::GridMap>(vis_topic_, 1, true);

  return true;
}

void HLUTHeuristicPlugin::preparePlanning(const l3_footstep_planning_msgs::StepPlanRequest& req)
{
  // get start floating base and position
  l3::FloatingBaseArray start_floating_bases;
  l3::floatingBaseArrayMsgToL3(req.start_floating_bases, start_floating_bases);
  start_fb_ = start_floating_bases.front();
  start_pos_ = l3::Position2D(start_fb_.x(), start_fb_.y());

  // get goal floating base and position
  l3::FloatingBaseArray goal_floating_bases;
  l3::floatingBaseArrayMsgToL3(req.goal_floating_bases, goal_floating_bases);
  goal_fb_ = goal_floating_bases.front();
  goal_pos_ = l3::Position2D(goal_fb_.x(), goal_fb_.y());

  hlut_ = initializeHLUT(start_fb_, goal_fb_, goal_pos_);

  // ensure that the is accessible cache is empty before starting the pre-computation
  is_accessible_.clear();

  // get start and goal index in the HLUT
  l3::PositionIndex start_index = hlut_.getIndexFromPosition(start_pos_);
  l3::PositionIndex goal_index = hlut_.getIndexFromPosition(goal_pos_);

  // min heap for quick access of first element
  std::priority_queue<hlutEntry, std::vector<hlutEntry>, gridMapHeuristicEntryCompare> considered_next;

  // insert goal index into the min heap
  considered_next.push({ goal_index, 0.0f });

  while (!considered_next.empty())
  {
    // get the first element from the min heap (the one with the lowest heuristic value)
    auto min_element = considered_next.top();
    considered_next.pop();

    l3::PositionIndex current_index = min_element.index;
    // if there is already a lower heuristic value for this index, skip it
    if (hlut_.getHeuristicEntry(current_index) <= min_element.heuristic_distance)
      continue;

    // insert the new heuristic value into the HLUT
    hlut_.setHeuristicEntry(current_index, min_element.heuristic_distance);

    // stop if the start index has been reached
    if (current_index == start_index)
      break;

    std::vector<l3::PositionIndex> valid_neighbors = getValidNeighbors(getNeighbors(current_index));

    // insert the neighbors with the new heuristic value into the min heap
    for (auto& valid_neighbor : valid_neighbors)
      considered_next.push(computeHLUTEntryOfNeighbor(valid_neighbor, min_element));
  }

  if (visualize_)
    visualizeHLUT();
}

double HLUTHeuristicPlugin::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  // get the floating bases
  FloatingBase::ConstPtr from_fb = from.getFloatingBase(l3::BaseInfo::MAIN_BODY_IDX);
  FloatingBase::ConstPtr to_fb = to.getFloatingBase(l3::BaseInfo::MAIN_BODY_IDX);

  return getHeuristicValue(*from_fb, *to_fb, start, goal);
}

double HLUTHeuristicPlugin::getHeuristicValue(const Foothold& from, const Foothold& /*to*/, const State& /*start*/, const State& /*goal*/) const
{
  // get the heuristic value from the HLUT for the given position
  return hlut_.getHeuristicEntry(l3::Position2D(from.x(), from.y()));
}

double HLUTHeuristicPlugin::getHeuristicValue(const FloatingBase& from, const FloatingBase& /*to*/, const State& /*start*/, const State& /*goal*/) const
{
  // get the heuristic value from the HLUT for the given position
  return hlut_.getHeuristicEntry(l3::Position2D(from.x(), from.y()));
}

HeuristicLookupTable HLUTHeuristicPlugin::initializeHLUT(const FloatingBase& start_fb, const FloatingBase& goal_fb, const l3::Position2D& goal_pos) const
{
  int size_x = size_x_;
  int size_y = size_y_;

  // check if start lies within the HLUT with the specified size.
  // if not, resize the HLUT
  if (euclideanDistance(start_fb.x(), 0.0, goal_fb.x(), 0.0) > size_x_ * 0.5)
  {
    size_x = 2 * static_cast<int>(std::ceil(euclideanDistance(start_fb.x(), 0.0, goal_fb.x(), 0.0))) + 1;
    ROS_WARN("[%s]: HLUT size_x is too small. Increasing to %d.", getName().c_str(), size_x);
  }
  if (euclideanDistance(0.0, start_fb.y(), 0.0, goal_fb.y()) > size_y_ * 0.5)
  {
    size_y = 2 * static_cast<int>(std::ceil(euclideanDistance(0.0, start_fb.y(), 0.0, goal_fb.y()))) + 1;
    ROS_WARN("[%s]: HLUT size_y is too small. Increasing to %d.", getName().c_str(), size_y);
  }

  // initialize HLUT
  return HeuristicLookupTable(resolution_, Eigen::Vector2i(size_x, size_y), goal_pos);
}

std::vector<l3::PositionIndex> HLUTHeuristicPlugin::getValidNeighbors(const std::vector<l3::PositionIndex>& neighbors) const
{
  std::vector<l3::PositionIndex> valid_neighbors = {};

  for (auto& neighbor : neighbors)
  {
    // check if the neighbor is already in the is accessible cache
    if (is_accessible_.count(neighbor) && is_accessible_.at(neighbor))
    {
      valid_neighbors.push_back(neighbor);
      continue;
    }

    // skip if the neighbor is not inside the HLUT
    if (!hlut_.isIndexInside(neighbor))
    {
      is_accessible_[neighbor] = false;
      continue;
    }

    // get the position of the neighbor
    l3::Position2D neighbor_pos = hlut_.getPositionFromIndex(neighbor);
    bool all_accessible = false;

    // check the foothold accessibility if the corresponding parameter is set to true
    if (check_foothold_accessibility_)
    {
      // check if the neighbor is accessible for all yaw values in angle_bins_
      for (auto& yaw : angle_bins_)
      {
        all_accessible = WorldModel::instance().isAccessible(l3::Foothold(neighbor_pos.x(), neighbor_pos.y(), 0.0, 0.0, 0.0, yaw));

        // stop if the neighbor is not accessible for at least one yaw value
        if (!all_accessible)
          break;
      }

      // continue with the next neighbor if the current one is not accessible for at least one yaw value
      if (!all_accessible)
      {
        is_accessible_[neighbor] = false;
        continue;
      }
    }

    // check the floating base accessibility if the corresponding parameter is set to true
    if (check_floating_base_accessibility_)
    {
      // check if the neighbor is accessible for all yaw values in angle_bins_
      for (auto& yaw : angle_bins_)
      {
        all_accessible = WorldModel::instance().isAccessible(l3::FloatingBase(neighbor_pos.x(), neighbor_pos.y(), 0.0, 0.0, 0.0, yaw));

        // stop if the neighbor is not accessible for at least one yaw value
        if (!all_accessible)
          break;
      }
      // continue with the next neighbor if the current one is not accessible for at least one yaw value
      if (!all_accessible)
      {
        is_accessible_[neighbor] = false;
        continue;
      }
    }

    // add the neighbor to the list of valid neighbors
    valid_neighbors.push_back(neighbor);

    is_accessible_[neighbor] = true;
  }

  return valid_neighbors;
}

void HLUTHeuristicPlugin::visualizeHLUT() const
{
  // publish HLUT
  grid_map::GridMap hlut_grid_map = grid_map::GridMap();
  hlut_grid_map.setFrameId(vis_frame_id_);
  hlut_grid_map.setGeometry(grid_map::Length(hlut_.getLength().x(), hlut_.getLength().y()), hlut_.getResolution().resolution().x, hlut_.getCenter());
  hlut_grid_map.add(vis_layer_, hlut_.getHeuristicMatrix());

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(hlut_grid_map, msg);
  hlut_pub_.publish(msg);
}
}  // namespace l3_footstep_planning
