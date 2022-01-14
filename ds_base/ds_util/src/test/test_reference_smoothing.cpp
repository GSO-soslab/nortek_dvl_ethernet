/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ds_util/ds_util.h"

#include <gtest/gtest.h>

// This test case should capture valid strings that we don't have parsers for
TEST(ReferenceSmoothing, sign)
{
  EXPECT_EQ(ds_util::sgn(3.45), 1.0);
  EXPECT_EQ(ds_util::sgn(-2.67), -1.0);
  EXPECT_EQ(ds_util::sgn(0.0), 1.0);
}

TEST(ReferenceSmoothing, trapezoidal)
{
  double pos, vel, acc;

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_EQ(pos, 0.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 0.0);

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_EQ(pos, 0.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 1.0);

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, pos, vel, acc, 1.0, 1.0, 1.0);
  EXPECT_EQ(pos, 1.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 0.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
