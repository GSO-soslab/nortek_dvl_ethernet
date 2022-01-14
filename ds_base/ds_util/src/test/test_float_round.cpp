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
//
// Created by jvaccaro on 3/27/18.
//

#include "ds_util/ds_util.h"
#include "ds_util/float_round.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class FloatRoundTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
  }
};

TEST_F(FloatRoundTest, round_zero)
{
  float flt = 0.0;
  EXPECT_FLOAT_EQ(0.0, ds_util::float_round(flt, 10));
}

TEST_F(FloatRoundTest, round_levels)
{
  float flt = 1.1234;
  EXPECT_FLOAT_EQ(1.1234, ds_util::float_round(flt, 4));
  EXPECT_FLOAT_EQ(1.123, ds_util::float_round(flt, 3));
  EXPECT_FLOAT_EQ(1.12, ds_util::float_round(flt, 2));
  EXPECT_FLOAT_EQ(1.1, ds_util::float_round(flt, 1));
  EXPECT_FLOAT_EQ(1, ds_util::float_round(flt, 0));
}

TEST_F(FloatRoundTest, round_up)
{
  EXPECT_FLOAT_EQ(2, ds_util::float_round(1.5, 0));
  EXPECT_FLOAT_EQ(1.2, ds_util::float_round(1.15, 1));
  EXPECT_FLOAT_EQ(1.12, ds_util::float_round(1.115, 2));
  EXPECT_FLOAT_EQ(2.0, ds_util::float_round(1.999, 2));
  EXPECT_FLOAT_EQ(1.3, ds_util::float_round(1.25, 1));
}

TEST_F(FloatRoundTest, double_round)
{
  EXPECT_FLOAT_EQ(2, ds_util::float_round(ds_util::float_round(1.45, 1), 0));
  EXPECT_FLOAT_EQ(1.1, ds_util::float_round(ds_util::float_round(1.115, 2), 1));
  EXPECT_FLOAT_EQ(1.46, ds_util::float_round(ds_util::float_round(1.4555, 3), 2));
  EXPECT_FLOAT_EQ(1.192, ds_util::float_round(ds_util::float_round(1.1919995, 5), 4));
  EXPECT_FLOAT_EQ(1.6869, ds_util::float_round(ds_util::float_round(1.68686841, 6), 4));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
