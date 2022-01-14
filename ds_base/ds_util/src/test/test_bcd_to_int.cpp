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
// Created by jvaccaro on 10/4/18.
//

#include "ds_util/ds_util.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Bcd2IntTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
  }
};

TEST_F(Bcd2IntTest, zero)
{
  uint16_t test = 0x00;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_35)
{
  uint16_t test = 0x35;
  EXPECT_EQ(35, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_max)
{
  int16_t test = 0x99;
  EXPECT_EQ(99, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_overflow)
{
  int16_t test = 0xAA;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_ones_overflow)
{
  int16_t test = 0x0A;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_tens_overflow)
{
  int16_t test = 0xA0;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
