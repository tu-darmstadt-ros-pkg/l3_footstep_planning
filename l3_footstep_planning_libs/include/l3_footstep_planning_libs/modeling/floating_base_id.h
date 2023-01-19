//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_FLOATING_BASE_ID_H__
#define L3_FOOTSTEP_PLANNING_FLOATING_BASE_ID_H__

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_libs/types/types.h>

#define FLOATING_BASE_ID_SIZE 7

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The FloatingBaseKey struct consists only of the main components uniquely identifying a
 * floating base within the (discretized) database
 */
struct FloatingBaseKey
{
  inline FloatingBaseKey()
    : FloatingBaseKey(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  {}

  inline FloatingBaseKey(const FloatingBase& base)
    : FloatingBaseKey(base.idx, base.x(), base.y(), base.z(), base.roll(), base.pitch(), base.yaw())
  {}

  inline FloatingBaseKey(FloatingBase::ConstPtr base)
    : FloatingBaseKey(*base)
  {}

  inline FloatingBaseKey(const FootIndex& foot_idx, const Pose& pose)
    : FloatingBaseKey(foot_idx, pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw())
  {}

  FloatingBaseKey(const BaseIndex& idx, double x, double y, double z, double roll, double pitch, double yaw)
    : idx(idx)
    , x(x)
    , y(y)
    , z(z)
    , roll(roll)
    , pitch(pitch)
    , yaw(yaw)
  {}

  BaseIndex idx;
  double x, y, z, roll, pitch, yaw;
};

typedef Eigen::Matrix<int, FLOATING_BASE_ID_SIZE, 1> EigenFloatingBaseID;

/**
 * @brief The FloatingBaseID representes a Foothold in discrete space.
 */
class FloatingBaseID : public EigenFloatingBaseID
{
public:
  FloatingBaseID()
    : EigenFloatingBaseID()
  {
    setZero();
  }

  FloatingBaseID(const EigenFloatingBaseID& vector)
    : EigenFloatingBaseID(vector)
  {
    hash_ = hash_value(*this);
  }

  inline FloatingBaseID(const FloatingBase& base, const DiscreteResolution& resolution)
    : FloatingBaseID(base.idx, base.x(), base.y(), base.z(), base.roll(), base.pitch(), base.yaw(), resolution)
  {}

  inline FloatingBaseID(const FloatingBaseKey& key, const DiscreteResolution& resolution)
    : FloatingBaseID(key.idx, key.x, key.y, key.z, key.roll, key.pitch, key.yaw, resolution)
  {}

  inline FloatingBaseID(const BaseIndex& idx, const Pose& pose, const DiscreteResolution& resolution)
    : FloatingBaseID(idx, pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw(), resolution)
  {}

  FloatingBaseID(const BaseIndex& foot_idx, double x, double y, double z, double roll, double pitch, double yaw, const DiscreteResolution& resolution)
  {
    getFloatingBaseId(*this, 0, foot_idx, x, y, z, roll, pitch, yaw, resolution);

    boost::hash<FloatingBaseID> hasher;
    hash_ = hasher(*this);
  }

  inline bool operator==(const FloatingBaseID& other) const { return hash_ == other.hash_ && static_cast<const EigenFloatingBaseID&>(*this) == static_cast<const EigenFloatingBaseID&>(other); }

  /** Specialization for boost::hash */
  friend Hash hash_value(const FloatingBaseID& id)
  {
    assert(id.size() == FLOATING_BASE_ID_SIZE);

    Hash seed = 0;
    for (long i = 0; i < FLOATING_BASE_ID_SIZE; i++)
      boost::hash_combine(seed, id(i));
    return seed;
  }

  /**
   * @brief Helper function that fills beginning at given start index a given vector with the foothold's id.
   */
  template <typename Scalar, int Rows>
  inline static void getFloatingBaseId(Eigen::Matrix<Scalar, Rows, 1>& m, const Eigen::Index& start_index, const BaseIndex& idx,
                                       double x, double y, double z, double roll, double pitch, double yaw,
                                       const DiscreteResolution& resolution)
  {
    m(start_index) = static_cast<int>(idx);
    m(start_index + 1) = resolution.toDiscX(x);
    m(start_index + 2) = resolution.toDiscY(y);
    m(start_index + 3) = resolution.toDiscZ(z);
    m(start_index + 4) = resolution.toDiscAngle(roll);
    m(start_index + 5) = resolution.toDiscAngle(pitch);
    m(start_index + 6) = resolution.toDiscAngle(yaw);
  }

  /**
   * @brief Returns the corresponding discretized values in continuous space.
   * @param resolution Resolution of discretized space (cell size)
   * @return Discretized foothold
   */
  FloatingBase getDiscreteFloatingBase(const DiscreteResolution& resolution) const
  {
    return FloatingBase(static_cast<BaseIndex>(coeffRef(0)),
                        resolution.toContX(coeffRef(1)), resolution.toContY(coeffRef(2)), resolution.toContZ(coeffRef(3)),
                        resolution.toContAngle(coeffRef(4)), resolution.toContAngle(coeffRef(5)), resolution.toContAngle(coeffRef(6)));
  }

  inline const Hash& getHashValue() const { return hash_; }

protected:
  Hash hash_;
};

typedef Hashed<FloatingBase, FloatingBaseID> FloatingBaseHashed;
typedef std::vector<FloatingBaseHashed::ConstPtr> FloatingBaseHashedConstPtrArray;
typedef Hasher<FloatingBaseID> FloatingBaseHasher;  // Hashing operator

/**
 * @brief Extension of hashing based on boost library.
 * @param array Input FootholdArray
 * @return calculated hash value
 */
Hash hash_value(const FloatingBaseHashedConstPtrArray& bases);

L3_STATIC_ASSERT_MOVEABLE(FloatingBaseKey)
L3_STATIC_ASSERT_MOVEABLE(FloatingBaseID)
L3_STATIC_ASSERT_MOVEABLE(FloatingBaseHashed)
}  // namespace l3_footstep_planning

#endif
