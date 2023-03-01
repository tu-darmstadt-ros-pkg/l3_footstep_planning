#include <l3_footstep_planning_libs/types/heuristic_lookup_table.h>

namespace l3_footstep_planning
{
HeuristicLookupTable::HeuristicLookupTable() {}

HeuristicLookupTable::HeuristicLookupTable(DiscreteResolution resolution, const Eigen::Vector2i& size, const l3::Position2D& center = l3::Position2D(0, 0))
{
  size_ = size;
  if (size_[0] <= 0 || size_[1] <= 0)
    ROS_ERROR("[HeuristicLookupTable] Size of the heuristic lookup must be greater than 0!");

  resolution_ = resolution;
  if (resolution_.resolution().x <= 0 || resolution_.resolution().y <= 0)
    ROS_ERROR("[HeuristicLookupTable] Resolution of the heuristic lookup table must be greater than 0!");

  center_ = center;
  center_index_ = PositionIndex(resolution.toDiscX(size_[0] * 0.5), resolution.toDiscY(size_[1] * 0.5));

  hlut_.resize(resolution.toDiscX(size_[0]), resolution.toDiscY(size_[1]));
  hlut_.setConstant(std::numeric_limits<float>::infinity());
}

void HeuristicLookupTable::setHeuristicEntry(const Position2D& pos, float value)
{
  PositionIndex index = getIndexFromPosition(pos);
  hlut_(index[0], index[1]) = value;
}

void HeuristicLookupTable::setHeuristicEntry(const PositionIndex& index, float value) { hlut_(index[0], index[1]) = value; }

float HeuristicLookupTable::getHeuristicEntry(const Position2D& pos) const
{
  PositionIndex index = getIndexFromPosition(pos);
  return hlut_(index[0], index[1]);
}

void HeuristicLookupTable::setHeuristicMatrix(const Eigen::MatrixXf& hlut) { hlut_ = hlut; }

float HeuristicLookupTable::getHeuristicEntry(const PositionIndex& index) const { return hlut_(index[0], index[1]); }

PositionIndex HeuristicLookupTable::getIndexFromPosition(const Position2D& pos) const
{
  return { resolution_.toDiscX(center_[0] - pos[0]) + center_index_[0], resolution_.toDiscY(center_[1] - pos[1]) + center_index_[1] };
}

l3::Position2D HeuristicLookupTable::getPositionFromIndex(const PositionIndex& index) const
{
  return { resolution_.toContX(center_index_[0] - index[0]) + center_[0], resolution_.toContY(center_index_[1] - index[1]) + center_[1] };
}

bool HeuristicLookupTable::isIndexInside(const PositionIndex& index) const { return index[0] >= 0 && index[0] < hlut_.rows() && index[1] >= 0 && index[1] < hlut_.cols(); }

Eigen::MatrixXf HeuristicLookupTable::getHeuristicMatrix() const { return hlut_; }

Eigen::Vector2i HeuristicLookupTable::getSize() const { return size_; }

DiscreteResolution HeuristicLookupTable::getResolution() const { return resolution_; }

l3::Position2D HeuristicLookupTable::getCenter() const { return center_; }
}  // namespace l3_footstep_planning
