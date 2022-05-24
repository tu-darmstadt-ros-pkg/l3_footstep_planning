//=================================================================================================
// Copyright (c) 2022, alexander stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_FLOATING_BASE_DATABASE_H__
#define L3_FOOTSTEP_PLANNING_FLOATING_BASE_DATABASE_H__

#include <unordered_map>

#include <l3_footstep_planning_libs/typedefs.h>
#include <l3_footstep_planning_libs/hash.h>

#include <l3_libs/types/types.h>
#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

namespace l3_footstep_planning
{
using namespace l3;

/**
 * @brief The FloatingBaseDataBase class
 */
class FloatingBaseDataBase
{
private:
  typedef std::unordered_map<FloatingBaseID, FloatingBaseHashed::ConstPtr, FloatingBaseHasher> FloatingBaseMap;

public:
  // typedefs
  typedef l3::SharedPtr<FloatingBaseDataBase> Ptr;
  typedef l3::SharedPtr<const FloatingBaseDataBase> ConstPtr;

  /**
   * @brief Discrete database for storing floating bases.
   * @param resolution Resolution [m/per cell] of discrete space
   */
  FloatingBaseDataBase(const DiscreteResolution& resolution);

  /**
   * @brief Returns discretized version of given input (continous space) floating bases when available.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return Pointer to discretized version of given input floating base when available; otherwise nullptr
   */
  template<typename KeyType>
  inline FloatingBaseHashed::ConstPtr operator()(const KeyType& key) const { return get(key); }

  /**
   * @brief Returns discretized version of given input (continous space) floating base when available.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return Pointer to discretized version of given input floating base when available; otherwise nullptr
   */
  template<typename KeyType>
  inline FloatingBaseHashed::ConstPtr operator[](const KeyType& key) const { return get(key); }

  /**
   * @brief Number of floating bases stored in the database.
   * @return Number of stored floating bases
   */
  size_t size() const;

  /**
   * @brief Checks if database is empty
   * @return True, if database is empty
   */
  bool empty() const;

  /**
   * @brief Clears database
   */
  void clear();

  /**
   * @brief Inserts given (continous space) floating base into database. The floating base is automatically
   * transformed to discretized version.
   * @param floating_base Floating base to be inserted
   * @return Pointer to discretized version of input floating base as stored in db
   */
  FloatingBaseHashed::ConstPtr insert(const FloatingBase& floating_base);
  FloatingBaseHashed::ConstPtr insert(FloatingBase::ConstPtr floating_base);

  /**
   * @brief Checks if discretized version of given input (continous space) floating base is already in database.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return true, if corresponding floating base is in database
   */
  template<typename KeyType>
  inline bool has(const KeyType& key) const { return has(FloatingBaseID(key, resolution_)); }

  /**
   * @brief Returns discretized version of given input (continous space) floating base when available.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   * @return Pointer to discretized version of given input floating base when available; otherwise nullptr
   */
  template<typename KeyType>
  inline FloatingBaseHashed::ConstPtr get(const KeyType& key) const { return get(FloatingBaseID(key, resolution_)); }

  /**
   * @brief Removes discretized version of given input (continous space) floating base.
   * @param KeyType type of input key that can be from any type, which can be also converted by FloatingBaseID
   * @param key input key to look up
   */
  template<typename KeyType>
  inline void remove(const KeyType& key) { remove(FloatingBaseID(key, resolution_)); }

  /**
   * @brief Returns statistical performance information. The database logs the number of successful
   * caching operations.
   * @return Number of successful hits
   */
  inline unsigned long hits() const { return hits_; }

  /**
   * @brief Returns statistical performance information. The database logs the number of successful
   * caching operations.
   * @return Number of misses
   */
  inline unsigned long miss() const { return miss_; }

  /**
   * @brief Returns resolution used for discretization by the database
   * @return Resolution of discretization grid
   */
  inline const DiscreteResolution& getResolution() const { return resolution_; }

protected:
  bool has(const FloatingBaseID& id) const;

  FloatingBaseHashed::ConstPtr get(const FloatingBaseID& id) const;

  void remove(const FloatingBaseID& id);

  mutable Mutex mutex_;

  DiscreteResolution resolution_;

  FloatingBaseMap map_;

  mutable std::atomic_ulong hits_;
  mutable std::atomic_ulong miss_;
};
}  // namespace l3_footstep_planning

#endif
