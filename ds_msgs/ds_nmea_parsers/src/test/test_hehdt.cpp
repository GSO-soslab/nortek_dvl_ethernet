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
#include "ds_nmea_parsers/Hdt.h"

#include <list>
#include <gtest/gtest.h>

TEST(HEHDT, valid_strings)
{

  auto hdt = [](double heading, bool true_flag, uint8_t checksum) {
    auto msg = ds_nmea_msgs::Hdt{};
    msg.heading = heading;
    msg.is_true = static_cast<unsigned char>(true_flag);
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::Hdt>>{
          {"$HEHDT,123.4,T*1F\r\n", hdt(123.4, true, 0x1F)},
          {"$HEHDT,284.49,T*1C", hdt(284.49, true, 0x1C)}
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::Hdt{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(expected.heading, msg.heading);
    EXPECT_EQ(expected.is_true, msg.is_true);
    EXPECT_EQ(expected.checksum, msg.checksum);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
