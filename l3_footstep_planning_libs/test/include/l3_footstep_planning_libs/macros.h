//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_FOOTSTEP_PLANNING_LIBS_TEST_MACROS_H__
#define L3_FOOTSTEP_PLANNING_LIBS_TEST_MACROS_H__

#include <gtest/gtest.h>

#define ANGULAR_PREC 1e-15

#define EXPECT_ANGLE_EQ(expected, actual) EXPECT_NEAR(normalizeAngle(expected), normalizeAngle(actual), ANGULAR_PREC);

#define EXPECT_VECTOR_EQ(expected, actual)                                                                                                                                         \
  {                                                                                                                                                                                \
    EXPECT_NEAR((expected).x(), (actual).x(), ANGULAR_PREC);                                                                                                                       \
    EXPECT_NEAR((expected).y(), (actual).y(), ANGULAR_PREC);                                                                                                                       \
    EXPECT_NEAR((expected).z(), (actual).z(), ANGULAR_PREC);                                                                                                                       \
  }

#define EXPECT_POSE_EQ(expected, actual)                                                                                                                                           \
  {                                                                                                                                                                                \
    EXPECT_VECTOR_EQ(expected, actual);                                                                                                                                            \
    EXPECT_ANGLE_EQ((expected).roll(), (actual).roll());                                                                                                                           \
    EXPECT_ANGLE_EQ((expected).pitch(), (actual).pitch());                                                                                                                         \
    EXPECT_ANGLE_EQ((expected).yaw(), (actual).yaw());                                                                                                                             \
  }

#define EXPECT_FOOTHOLD_EQ(expected, actual)                                                                                                                                       \
  {                                                                                                                                                                                \
    EXPECT_EQ((expected).idx, (actual).idx);                                                                                                                                       \
    EXPECT_VECTOR_EQ(expected, actual);                                                                                                                                            \
    EXPECT_ANGLE_EQ((expected).roll(), (actual).roll());                                                                                                                           \
    EXPECT_ANGLE_EQ((expected).pitch(), (actual).pitch());                                                                                                                         \
    EXPECT_ANGLE_EQ((expected).yaw(), (actual).yaw());                                                                                                                             \
    EXPECT_EQ((expected).header, (actual).header);                                                                                                                                 \
    EXPECT_EQ((expected).data.size(), (actual).data.size());                                                                                                                       \
  }

#define EXPECT_FLOATING_BASE_EQ(expected, actual)                                                                                                                                  \
  {                                                                                                                                                                                \
    EXPECT_EQ((expected).idx, (actual).idx);                                                                                                                                       \
    EXPECT_VECTOR_EQ(expected, actual);                                                                                                                                            \
    EXPECT_ANGLE_EQ((expected).roll(), (actual).roll());                                                                                                                           \
    EXPECT_ANGLE_EQ((expected).pitch(), (actual).pitch());                                                                                                                         \
    EXPECT_ANGLE_EQ((expected).yaw(), (actual).yaw());                                                                                                                             \
    EXPECT_EQ((expected).header, (actual).header);                                                                                                                                 \
    EXPECT_EQ((expected).data.size(), (actual).data.size());                                                                                                                       \
  }

#define EXPECT_FOOTHOLD_KEY(_idx, _x, _y, _z, _yaw, actual)                                                                                                                        \
  {                                                                                                                                                                                \
    EXPECT_EQ(_idx, (actual).idx);                                                                                                                                                 \
    EXPECT_EQ(_x, (actual).x);                                                                                                                                                     \
    EXPECT_EQ(_y, (actual).y);                                                                                                                                                     \
    EXPECT_EQ(_z, (actual).z);                                                                                                                                                     \
    EXPECT_EQ(_yaw, (actual).yaw);                                                                                                                                                 \
  }

#define EXPECT_FOOTHOLD_ID(id, x, y, z, yaw, actual)                                                                                                                               \
  {                                                                                                                                                                                \
    EXPECT_EQ(id, (actual).coeff(0));                                                                                                                                              \
    EXPECT_EQ(x, (actual).coeff(1));                                                                                                                                               \
    EXPECT_EQ(y, (actual).coeff(2));                                                                                                                                               \
    EXPECT_EQ(z, (actual).coeff(3));                                                                                                                                               \
    EXPECT_EQ(yaw, (actual).coeff(4));                                                                                                                                             \
  }

#define EXPECT_FLOATING_BASE_ID(id, x, y, z, roll, pitch, yaw, actual)                                                                                                             \
  {                                                                                                                                                                                \
    EXPECT_EQ(id, (actual).coeff(0));                                                                                                                                              \
    EXPECT_EQ(x, (actual).coeff(1));                                                                                                                                               \
    EXPECT_EQ(y, (actual).coeff(2));                                                                                                                                               \
    EXPECT_EQ(z, (actual).coeff(3));                                                                                                                                               \
    EXPECT_EQ(roll, (actual).coeff(4));                                                                                                                                            \
    EXPECT_EQ(pitch, (actual).coeff(5));                                                                                                                                           \
    EXPECT_EQ(yaw, (actual).coeff(6));                                                                                                                                             \
  }

#define EXPECT_STATE_ID(expected, actual)                                                                                                                                          \
  {                                                                                                                                                                                \
    EXPECT_EQ((expected).rows(), (actual).rows());                                                                                                                                 \
    for (size_t i = 0; i < static_cast<size_t>((actual).rows()); i++)                                                                                                              \
      EXPECT_EQ((expected).coeff(static_cast<long>(i)), (actual).coeff(static_cast<long>(i)));                                                                                     \
  }

#define EXPECT_DB_ENTRY_EQ(expected, db, key)                                                                                                                                      \
  {                                                                                                                                                                                \
    EXPECT_EQ(expected, (db)((key)));                                                                                                                                              \
    EXPECT_EQ(expected, (db)[(key)]);                                                                                                                                              \
    EXPECT_EQ(expected, (db).get((key)));                                                                                                                                          \
  }

#define EXPECT_DB_DOUBLE_ENTRY_EQ(expected, db, key1, key2)                                                                                                                        \
  {                                                                                                                                                                                \
    EXPECT_EQ(expected, (db)((key1), (key2)));                                                                                                                                     \
    EXPECT_EQ(expected, (db).get((key1), (key2)));                                                                                                                                 \
  }

#endif
