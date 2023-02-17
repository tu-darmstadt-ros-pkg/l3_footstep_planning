#include <l3_footstep_planning_plugins/std/heuristic/fast_marching_method_heuristic.h>

namespace l3_footstep_planning
{
FastMarchingMethodHeuristic::FastMarchingMethodHeuristic()
  : HLUTHeuristicPlugin("fast_marching_method_heuristic")
{}

std::vector<l3::PositionIndex> FastMarchingMethodHeuristic::getNeighbors(const l3::PositionIndex& current_index) const
{
  l3::PositionIndex up = current_index;
  up[0]++;
  l3::PositionIndex down = current_index;
  down[0]--;
  l3::PositionIndex left = current_index;
  left[1]--;
  l3::PositionIndex right = current_index;
  right[1]++;

  return { up, down, left, right };
}

HLUTHeuristicPlugin::hlutEntry FastMarchingMethodHeuristic::computeHLUTEntryOfNeighbor(const l3::PositionIndex& neighbor, const hlutEntry& /*current_entry*/) const
{
  l3::PositionIndex neighbor_up = neighbor;
  neighbor_up[0]++;
  l3::PositionIndex neighbor_down = neighbor;
  neighbor_down[0]--;
  l3::PositionIndex neighbor_left = neighbor;
  neighbor_left[1]--;
  l3::PositionIndex neighbor_right = neighbor;
  neighbor_right[1]++;

  float a =
      std::min(hlut_.isIndexInside(neighbor_up) ? hlut_.getHeuristicEntry(neighbor_up) : inf, hlut_.isIndexInside(neighbor_down) ? hlut_.getHeuristicEntry(neighbor_down) : inf);
  float b = std::min(hlut_.isIndexInside(neighbor_left) ? hlut_.getHeuristicEntry(neighbor_left) : inf,
                     hlut_.isIndexInside(neighbor_right) ? hlut_.getHeuristicEntry(neighbor_right) : inf);

  // TODO: Allow speed functions
  float inv_f = 1.0f / static_cast<float>(resolution_.resolution().x);

  hlutEntry valid_neighbor_entry;
  valid_neighbor_entry.index = neighbor;

  if (inv_f > abs(a - b))
  {
    float c = a + b;
    valid_neighbor_entry.heuristic_distance = 0.5f * (c + sqrt(c * c - 2.0f * (a * a + b * b - inv_f * inv_f)));
  }
  else
  {
    valid_neighbor_entry.heuristic_distance = std::min(a, b) + inv_f;
  }

  return valid_neighbor_entry;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::FastMarchingMethodHeuristic, l3_footstep_planning::HeuristicPlugin)
