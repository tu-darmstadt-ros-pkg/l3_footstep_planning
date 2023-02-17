#include <l3_footstep_planning_plugins/std/heuristic/dijkstra_heuristic.h>

namespace l3_footstep_planning
{
DijkstraHeuristic::DijkstraHeuristic()
  : HLUTHeuristicPlugin("dijkstra_heuristic")
{}

bool DijkstraHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HLUTHeuristicPlugin::loadParams(params))
    return false;

  getParam("neighbor_costs_vector", neighbor_costs_vector_, { -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f });

  // compute the dimension of the neighbor_cost_matrix_
  neighbors_dimension_ = sqrt(neighbor_costs_vector_.size());

  if (neighbors_dimension_ != floor(neighbors_dimension_) || static_cast<int>(neighbors_dimension_) % 2 == 0 || neighbors_dimension_ < 3)
  {
    ROS_ERROR("[%s]: size of neighbor_costs_vector_ must be an odd square number greater than or equal to 9", getName().c_str());
    return false;
  }

  half_neighbors_dimension_ = static_cast<int>((neighbors_dimension_ - 1) * 0.5);

  // insert neighbor_costs_vector_ into neighbor_costs_matrix_
  for (int i = 0; i < neighbors_dimension_; i++)
  {
    std::vector<float> row;
    for (int j = 0; j < neighbors_dimension_; j++)
    {
      row.push_back(neighbor_costs_vector_[i * static_cast<int>(neighbors_dimension_) + j]);
    }
    neighbor_costs_matrix_.push_back(row);
  }

  return true;
}

std::vector<l3::PositionIndex> DijkstraHeuristic::getNeighbors(const l3::PositionIndex& current_index) const
{
  std::vector<l3::PositionIndex> neighbors;

  for (int i = 0; i < neighbors_dimension_; i++)
  {
    for (int j = 0; j < neighbors_dimension_; j++)
    {
      if (neighbor_costs_matrix_[i][j] >= 0)
        neighbors.emplace_back(current_index + l3::PositionIndex(i - half_neighbors_dimension_, j - half_neighbors_dimension_));
    }
  }

  return neighbors;
}

HLUTHeuristicPlugin::hlutEntry DijkstraHeuristic::computeHLUTEntryOfNeighbor(const l3::PositionIndex& neighbor, const hlutEntry& current_entry) const
{
  return { neighbor,
           current_entry.heuristic_distance +
               neighbor_costs_matrix_[neighbor[0] - current_entry.index[0] + half_neighbors_dimension_][neighbor[1] - current_entry.index[1] + half_neighbors_dimension_] };
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::DijkstraHeuristic, l3_footstep_planning::HeuristicPlugin)
