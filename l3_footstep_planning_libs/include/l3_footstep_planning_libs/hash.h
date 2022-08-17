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

#ifndef L3_FOOTSTEP_PLANNING_HASH_H__
#define L3_FOOTSTEP_PLANNING_HASH_H__

#include <l3_libs/types/typedefs.h>
#include <l3_footstep_planning_libs/typedefs.h>

namespace l3
{
using namespace l3_footstep_planning;

/**
 *  Wrapper class in order to add an ID to an object.
 *  TID: Non-unique id which describes the object (e.g. usable as key for a std::unordered_map)
 *  UID: Unique id which identifies the object
 */
template <typename T, typename TID = int>
class Hashed : public T
{
public:
  // typedefs
  typedef l3::SharedPtr<Hashed<T, TID>> Ptr;
  typedef l3::SharedPtr<const Hashed<T, TID>> ConstPtr;

  /** Default constructor */
  Hashed()
    : hash_(0)
    , id_(TID())
    , uid_(-1)
  {}

  /** Full constructor */
  Hashed(const T& input, const TID& id, const UID& uid = 0)
    : T(input)
    , hash_(id.getHashValue())
    , id_(id)
    , uid_(uid)
  {}

  Hashed(T&& input, const TID& id, const UID& uid = 0)
    : T(std::move(input))
    , hash_(id.getHashValue())
    , id_(id)
    , uid_(uid)
  {}

  Hashed(const T& input, const UID& uid = 0)
    : Hashed(input, TID(), uid)
  {}

  Hashed(T&& input, const UID& uid = 0)
    : Hashed(std::move(input), TID(), uid)
  {}

  /** \brief Comparison Operator for Hashed datatypes */
  friend bool operator==(const Hashed& a, const Hashed& b)
  {
    return a.hash_ == b.hash_ && a.id_ == b.id_ && a.uid_ == b.uid_ && static_cast<const T&>(a) == static_cast<const T&>(b);
  }

  /** Specialization for boost::hash in order to used cached value */
  friend Hash hash_value(const Hashed& obj) { return obj.getHashValue(); }

  inline static Ptr make(const T& input, const TID& id, const UID& uid = 0) { return makeShared<Hashed>(input, id, uid); }
  inline static Ptr make(T&& input, const TID& id, const UID& uid = 0) { return makeShared<Hashed>(std::move(input), id, uid); }
  inline static Ptr make(const T& input, const UID& uid = 0) { return makeShared<Hashed>(input, uid); }
  inline static Ptr make(T&& input, const UID& uid = 0) { return makeShared<Hashed>(std::move(input), uid); }

  inline const Hash& getHashValue() const { return hash_; }

  const TID& getID() const { return id_; }

  void setUID(const UID& uid) { uid_ = uid; }
  const UID& getUID() const { return uid_; }

protected:
  Hash hash_;  // Hash value associated with this data
  TID id_;     // Specific ID associated with this data (e.g. lookup key for maps)
  UID uid_;    // Unique identifier associated with this data
};

template <typename T>
struct Hasher
{
  inline Hash operator()(const T& id) const { return id.getHashValue(); }
};

/**
 * @brief Extension of hashing based on boost library.
 * Note: The components roll and pitch are ignored as they are constrained by terrain.
 * @param foothold Input Foothold
 * @return calculated hash value
 */
Hash hash_value(const Foothold& foothold);

/**
 * @brief Extension of hashing based on boost library.
 * Note: The components roll and pitch are ignored as they are constrained by terrain.
 * @param array Input FootholdArray
 * @return calculated hash value
 */
Hash hash_value(const FootholdArray& array);

/**
 * @brief Extension of hashing based on boost library.
 * Note: The components roll and pitch are ignored as they are constrained by terrain.
 * @param floating_base Input FloatingBase
 * @return calculated hash value
 */
Hash hash_value(const FloatingBase& floating_base);

/**
 * @brief Extension of hashing based on boost library.
 * Note: The components roll and pitch are ignored as they are constrained by terrain.
 * @param array Input FloatingBaseArray
 * @return calculated hash value
 */
Hash hash_value(const FloatingBaseArray& array);
}  // namespace l3

#endif
