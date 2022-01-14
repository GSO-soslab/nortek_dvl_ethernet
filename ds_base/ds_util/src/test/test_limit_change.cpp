/**
* Copyright 2019 Woods Hole Oceanographic Institution
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
// Created by ivaughn on 3/14/19.
//

#include "ds_util/limit_change.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace ds_util;

constexpr static double DTOR = M_PI/180.0;
constexpr static double RTOD = 180.0/M_PI;

TEST(LimitChange, basic) {
  // limit low
  EXPECT_DOUBLE_EQ(-1.0, limit_change(-2.0, 2.0, 1.0));
  EXPECT_DOUBLE_EQ(-1.0, limit_change(-1.000001, 2.0, 1.0));

  // exactly match
  EXPECT_DOUBLE_EQ(-1.0, limit_change(-1.0, 2.0, 1.0));

  // accept goal
  EXPECT_DOUBLE_EQ(0.0, limit_change(0.0, 2.0, 1.0));
  EXPECT_DOUBLE_EQ(1.0, limit_change(1.0, 2.0, 1.0));
  EXPECT_DOUBLE_EQ(2.0, limit_change(2.0, 2.0, 1.0));

  // exactly match
  EXPECT_DOUBLE_EQ(3.0, limit_change(3.0, 2.0, 1.0));

  // limit high
  EXPECT_DOUBLE_EQ(3.0, limit_change(3.000001, 2.0, 1.0));
  EXPECT_DOUBLE_EQ(3.0, limit_change(5.0, 2.0, 1.0));
}

TEST(LimitChangeHeading, basic) {
    // check far side
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(-159.999999*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(200.000001*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // limit low
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(-2.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(358.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(-1.000001*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(358.999999*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // exact match
  EXPECT_DOUBLE_EQ(10.0, limit_change_heading(10.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // accept
  EXPECT_DOUBLE_EQ(10.1, limit_change_heading(10.1*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(15.0, limit_change_heading(15.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(20.0, limit_change_heading(20.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(25.0, limit_change_heading(25.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(29.9, limit_change_heading(29.9*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // exact match
  EXPECT_DOUBLE_EQ(30.0, limit_change_heading(30.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // limit high
  EXPECT_DOUBLE_EQ(30.0, limit_change_heading(30.0000001*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(30.0, limit_change_heading(60.0*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);

  // check far side
  EXPECT_DOUBLE_EQ(30.0, limit_change_heading(199.999999*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
  EXPECT_DOUBLE_EQ(30.0, limit_change_heading(-160.000001*DTOR, 10.0*DTOR, 20.0*DTOR)*RTOD);
}

TEST(LimitChangeHeading, at0) {
  // check far side
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(-179.999999*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(180.000001*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // limit low
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(-2.0*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(358.0*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(-1.000001*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(358.999999*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // exact match
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(-1.0*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.0, limit_change_heading(359.0*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // accept
  EXPECT_DOUBLE_EQ(359.1, limit_change_heading(-0.9*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(359.1, limit_change_heading(359.1*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(0.0, limit_change_heading(0.0*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(0.9, limit_change_heading(0.9*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // exact match
  EXPECT_DOUBLE_EQ(1.0, limit_change_heading(1.0*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // limit high
  EXPECT_DOUBLE_EQ(1.0, limit_change_heading(1.0000001*DTOR, 1.0*DTOR, 0.0)*RTOD);
  EXPECT_DOUBLE_EQ(1.0, limit_change_heading(2.0*DTOR, 1.0*DTOR, 0.0)*RTOD);

  // check far side
  EXPECT_DOUBLE_EQ(1.0, limit_change_heading(179.999999*DTOR, 1.0*DTOR, 0.0)*RTOD);
}


// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}