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
// Created by zac on 12/7/17.
//

#include <gtest/gtest.h>

#include <list>
extern auto parse_serial_connection(const std::string& arg) -> std::vector<std::string>;

// This test case should capture known-bad strings that should fail parsing.
TEST(ConnectionArgumentParsing, failingSerial)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto lines = std::list<std::string>{ "/dev/ttyUSB0,8N1", ":8N1", "/dev/ttyUSB0:8N" };

  for (const auto& line : lines)
  {
    // Attempt to parse
    auto result = parse_serial_connection(line);
    EXPECT_TRUE(result.empty());
  }
}

#if 0
TEST_F(AnderaaOxyOptodeTest, validParses) {


  // Simple function to create a ds_sensors_msg::OxygenConcentration from arguments
  auto oxy = [](float conc, float sat, float temp) {

    auto msg = ds_sensor_msgs::OxygenConcentration{};
    msg.concentration = conc;
    msg.air_saturation = sat;
    msg.temperature = temp;
    return msg;
  };

  // Add new test cases here.  These should pass
  const auto test_pairs = std::list<std::pair<std::string, ds_sensor_msgs::OxygenConcentration>> {
      {
          "MEASUREMENT 4330    20      O2Concentration(uM)     155.469 AirSaturation(%)        36.054  Temperature(Deg.C)      2.024   CalPhase(Deg)   45.442  TCPhase(Deg)    44.868  C1RPh(Deg) 48.889  C2RPh(Deg)      4.021   C1Amp(mV)       1041.7  C2Amp(mV)       808.9   RawTemp(mV)     691.1\r\n",
          oxy(155.469, 36.054, 2.024)
      },
      {
          "MEASUREMENT 4330    20      O2Concentration(uM)     156.963 AirSaturation(%)        36.827  Temperature(Deg.C)      2.454   CalPhase(Deg)   45.063  TCPhase(Deg)    44.490  C1RPh(Deg) 48.510  C2RPh(Deg)      4.020   C1Amp(mV)       1035.7  C2Amp(mV)       806.2   RawTemp(mV)     678.0\r\n",
          oxy(156.963, 36.827, 2.454)
      }
  };

  // Loop through all provided cases
  for(const auto& test_pair: test_pairs) {
    auto test_str = test_pair.first;
    auto test_ctd = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto ctd = ds_sensor_msgs::OxygenConcentration{};

    std::tie(ok, ctd) = ds_sensors::AnderaaOxyOptode::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), ctd.header.io_time.toSec());
    EXPECT_FLOAT_EQ(test_ctd.temperature, ctd.temperature);
    EXPECT_FLOAT_EQ(test_ctd.concentration, ctd.concentration);
    EXPECT_FLOAT_EQ(test_ctd.air_saturation, ctd.air_saturation);
  }
}
#endif

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
