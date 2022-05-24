//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_FOOTHOLD_ID_H__
#define L3_FOOTSTEP_PLANNING_FOOTHOLD_ID_H__

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_libs/types/types.h>

#define FOOTHOLD_ID_SIZE 5

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The FootholdKey struct consists only of the main components uniquely identifying a
 * foothold within the (discretized) database.
 */
struct FootholdKey
{
  inline FootholdKey()
    : FootholdKey(0, 0.0, 0.0, 0.0, 0.0)
  {}

  inline FootholdKey(const Foothold& foothold)
    : FootholdKey(foothold.idx, foothold.x(), foothold.y(), foothold.z(), foothold.yaw())
  {}

  inline FootholdKey(Foothold::ConstPtr foothold)
    : FootholdKey(*foothold)
  {}

  inline FootholdKey(const FootIndex& foot_idx, const Pose& pose)
    : FootholdKey(foot_idx, pose.x(), pose.y(), pose.z(), pose.yaw())
  {}

  FootholdKey(const FootIndex& idx, double x, double y, double z, double yaw)
    : idx(idx)
    , x(x)
    , y(y)
    , z(z)
    , yaw(yaw)
  {}

  FootIndex idx;
  double x, y, z, yaw;
};

typedef Eigen::Matrix<int, FOOTHOLD_ID_SIZE, 1> EigenFootholdID;

/**
 * @brief The FootholdId representes a Foothold in discrete space.
 */
class FootholdID : public EigenFootholdID
{
public:
  FootholdID()
    : EigenFootholdID()
  {
    setZero();
  }

  FootholdID(const EigenFootholdID& vector)
    : EigenFootholdID(vector)
  {
    hash_ = hash_value(*this);
  }

  inline FootholdID(const FootholdKey& key, const DiscreteResolution& resolution)
    : FootholdID(key.idx, key.x, key.y, key.z, key.yaw, resolution)
  {}

  inline FootholdID(const Foothold& foothold, const DiscreteResolution& resolution)
    : FootholdID(foothold.idx, foothold.x(), foothold.y(), foothold.z(), foothold.yaw(), resolution)
  {}

  inline FootholdID(const FootIndex& foot_idx, const Pose& pose, const DiscreteResolution& resolution)
    : FootholdID(foot_idx, pose.x(), pose.y(), pose.z(), pose.yaw(), resolution)
  {}

  FootholdID(const FootIndex& foot_idx, double x, double y, double z, double yaw, const DiscreteResolution& resolution)
  {
    getFoodholdId(*this, 0, foot_idx, x, y, z, yaw, resolution);

    boost::hash<FootholdID> hasher;
    hash_ = hasher(*this);
  }

  inline bool operator==(const FootholdID& other) const { return hash_ == other.hash_ && static_cast<const EigenFootholdID&>(*this) == static_cast<const EigenFootholdID&>(other); }

  /** Specialization for boost::hash */
  friend Hash hash_value(const FootholdID& id)
  {
    assert(id.size() == FOOTHOLD_ID_SIZE);

    Hash seed = 0;
    for (long i = 0; i < FOOTHOLD_ID_SIZE; i++)
      boost::hash_combine(seed, id(i));
    return seed;
  }

  /**
   * @brief Helper function that fills beginning at given start index a given vector with the foothold's id.
   */
  template <typename Scalar, int Rows>
  inline static void getFoodholdId(Eigen::Matrix<Scalar, Rows, 1>& m, const Eigen::Index& start_index, const FootIndex& idx, double x, double y, double z, double yaw,
                                   const DiscreteResolution& resolution)
  {
    m(start_index) = static_cast<int>(idx);
    m(start_index + 1) = resolution.toDiscX(x);
    m(start_index + 2) = resolution.toDiscY(y);
    m(start_index + 3) = resolution.toDiscZ(z);
    m(start_index + 4) = resolution.toDiscAngle(yaw);
  }

  /**
   * @brief Returns the corresponding discretized values in continuous space.
   * @param resolution Resolution of discretized space (cell size)
   * @return Discretized foothold
   */
  Foothold getDiscreteFoothold(const DiscreteResolution& resolution) const
  {
    return Foothold(static_cast<FootIndex>(coeffRef(0)), resolution.toContX(coeffRef(1)), resolution.toContY(coeffRef(2)), resolution.toContZ(coeffRef(3)), 0.0, 0.0, resolution.toContAngle(coeffRef(4)));
  }

  inline const Hash& getHashValue() const { return hash_; }

private:
  Hash hash_;
};

typedef Hashed<Foothold, FootholdID> FootholdHashed;
typedef std::vector<FootholdHashed::ConstPtr> FootholdHashedConstPtrArray;
typedef Hasher<FootholdID> FootholdHasher;  // Hashing operator

/**
 * @brief Extension of hashing based on boost library.
 * Note: The components roll and pitch are ignored as they are constrained by terrain.
 * @param array Input FootholdArray
 * @return calculated hash value
 */
Hash hash_value(const FootholdHashedConstPtrArray& footholds);

L3_STATIC_ASSERT_MOVEABLE(FootholdKey)
L3_STATIC_ASSERT_MOVEABLE(FootholdID)
L3_STATIC_ASSERT_MOVEABLE(FootholdHashed)
}  // namespace l3_footstep_planning

#endif
