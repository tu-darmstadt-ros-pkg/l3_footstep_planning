//=================================================================================================
// Copyright (c) 2023, Simon Giegerich, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_FOOTSTEP_PLANNING_LIBS_HEURISTIC_LOOKUP_TABLE_H
#define L3_FOOTSTEP_PLANNING_LIBS_HEURISTIC_LOOKUP_TABLE_H

#include <Eigen/Core>

#include <l3_libs/types/eigen_types.h>
#include <l3_footstep_planning_libs/typedefs.h>

namespace l3_footstep_planning
{
using namespace l3;

typedef Eigen::Vector2i PositionIndex;

/**
 * @brief A lookup table for the heuristic function.
 */
class HeuristicLookupTable
{
public:
  /**
   * @brief Default constructor.
   */
  HeuristicLookupTable();

  /**
   * @brief Constructor.
   * @param resolution The resolution of the heuristic lookup table.
   * @param size The x and y size of the heuristic lookup table.
   * @param center The position that lies in the center of the heuristic lookup table.
   */
  HeuristicLookupTable(DiscreteResolution resolution, const Eigen::Vector2i& size, const l3::Position2D& center);

  /**
   * @brief Set the heuristic value at a given position.
   * @param pos The position where the heuristic value should be set.
   * @param value The value to set.
   */
  void setHeuristicEntry(const l3::Position2D& pos, float value);

  /**
   * @brief Set the heuristic value at a given index.
   * @param index The index where the heuristic value should be set.
   * @param value The value to set.
   */
  void setHeuristicEntry(const PositionIndex& index, float value);

  /**
   * @brief Set the matrix containing the heuristic values.
   * @param hlut The matrix containing the heuristic values.
   */
  void setHeuristicMatrix(const Eigen::MatrixXf& hlut);

  /**
   * @brief Get the heuristic value at a given position.
   * @param pos The position where the heuristic value should be returned.
   * @return The heuristic value at the given position.
   */
  float getHeuristicEntry(const l3::Position2D& pos) const;

  /**
   * @brief Get the heuristic value at a given index.
   * @param index The index where the heuristic value should be returned.
   * @return The heuristic value at the given index.
   */
  float getHeuristicEntry(const PositionIndex& index) const;

  /**
   * @brief Get the index of a given position.
   * @param pos The position where the corresponding index should be returned.
   * @return The corresponding index of the given position.
   */
  PositionIndex getIndexFromPosition(const l3::Position2D& pos) const;

  /**
   * @brief Get the position of a given index.
   * @param index The index where the corresponding position should be returned.
   * @return The corresponding position of the given index.
   */
  l3::Position2D getPositionFromIndex(const PositionIndex& index) const;

  /**
   * @brief Specifies whether the given index is inside the heuristic lookup table.
   * @param index The index to check.
   * @return True if the index is inside of the heuristic lookup table, false otherwise.
   */
  bool isIndexInside(const PositionIndex& index) const;

  /**
   * @brief Get the matrix containing the heuristic values.
   * @return The matrix containing the heuristic values.
   */
  Eigen::MatrixXf getHeuristicMatrix() const;

  /**
   * @brief Get the size of the heuristic lookup table.
   * @return The size of the heuristic lookup table.
   */
  Eigen::Vector2i getSize() const;

  /**
   * @brief Get the resolution of the heuristic lookup table.
   * @return The resolution of the heuristic lookup table.
   */
  DiscreteResolution getResolution() const;

  /**
   * @brief Get the center of the heuristic lookup table.
   * @return The center of the heuristic lookup table.
   */
  l3::Position2D getCenter() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

protected:
  // The matrix representing the heuristic lookup table.
  Eigen::MatrixXf hlut_;

  // The x and y size of the heuristic lookup table.
  Eigen::Vector2i size_;

  // The resolution of the heuristic lookup table.
  DiscreteResolution resolution_;

  // The position that lies in the center of the heuristic lookup table.
  l3::Position2D center_;

  // The index of the center position.
  PositionIndex center_index_;
};
}  // namespace l3_footstep_planning

#endif  // L3_FOOTSTEP_PLANNING_LIBS_HEURISTIC_LOOKUP_TABLE_H
