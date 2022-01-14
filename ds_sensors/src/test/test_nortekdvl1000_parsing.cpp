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
// Created by ivandor on 7/25/19.
//
#include "ds_sensors/nortekdvl1000.h"
#include "../ds_sensors/nortekdvl1000_private.h"

#include <list>
#include <gtest/gtest.h>
#include <ros/time.h>
#include "nortekdvl_test_data.h"

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class NortekDvlTest : public ::testing::Test
{
public:
    double beam_angle = 30.0 * M_PI / 180.0;
    bool phased_array = false;
    double max_clock_offset = 1.0;


    // This method runs ONCE before a text fixture is run (not once-per-test-case)
    static void SetUpTestCase()
    {
        ros::Time::init();
    }

    // take one of the test datagram literals from nortekdvl_test_data.h and wrap it
    // in a nice RawData message
    ds_core_msgs::RawData buildRaw(const std::vector<uint8_t>& raw, size_t offset=0, size_t size=217) {
        ds_core_msgs::RawData ret;
        ret.header.stamp = ros::Time::now();
        ret.ds_header.io_time = ret.header.stamp;
        ret.data_direction = ds_core_msgs::RawData::DATA_IN;
        ret.data.resize(size);
        std::memcpy(ret.data.data(), raw.data()+offset, size);

        return ret;
    }
};

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(NortekDvlTest, totalParsePASS)
{
auto ok = false;
auto parsed_msg = ds_sensor_msgs::Dvl{};
auto df21 = ds_sensor_msgs::NortekDF21{};
auto rng = ds_sensor_msgs::Ranges3D{};
auto byte_msg = buildRaw(nortekdvl_test_data::pass);
std::tie(ok, df21) = ds_sensors::NortekDvl::parse_bytes(byte_msg, beam_angle, phased_array, max_clock_offset);
EXPECT_TRUE(ok);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(NortekDvlTest, headerParsePASS)
{
bool result = false;
auto byte_msg = buildRaw(nortekdvl_test_data::pass);

auto *hdr = reinterpret_cast<const ds_sensors::nortekdvl_structs::header*>(byte_msg.data.data());
result = true;
EXPECT_TRUE(result);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Example fail test case
TEST_F(NortekDvlTest, failingParses)
{
  ROS_ERROR_STREAM("Failing by parsing");

  const auto test_nums =
      std::list<int>{
          1, // start offset from header
          // INFO: ID not recognized

          3, // start with a non-starting valid header 0000
          // INFO: ID not recognized

          107, // start with header in the middle of a packet
          // Should parse header, then fail without sending message
          // INFO: too many data, too many bytes, and checksum failure

          218 // start with header at beginning, insert bad address value
          // Should parse header, then fail when seeking packets
          // INFO: too many data, checksum failure, address outside of buffer length

      };

  for (const auto& test_num : test_nums)
  {
    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_df21 = ds_sensor_msgs::NortekDF21{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};

    auto byte_msg = buildRaw(nortekdvl_test_data::fail, test_num);

    std::tie(ok, parsed_df21) = ds_sensors::NortekDvl::parse_bytes(byte_msg, beam_angle, phased_array, max_clock_offset);

// Should have failed
    EXPECT_FALSE(ok);
  }
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(NortekDvlTest, zerolength)
{
ROS_ERROR_STREAM("Failing by zerolength");

auto ok = true;
auto parsed_msg = ds_sensor_msgs::Dvl{};
auto parsed_df21 = ds_sensor_msgs::NortekDF21{};
auto parsed_rng = ds_sensor_msgs::Ranges3D{};

// Construct a ByteSequence message
auto byte_msg = ds_core_msgs::RawData{};
ROS_ERROR_STREAM("Byte msg length:" << byte_msg.data.size());
byte_msg.data.data();
ROS_ERROR_STREAM("Passed byte_msg.data.data()");

byte_msg.ds_header.io_time = ros::Time::now();
  std::tie(ok, parsed_df21) = ds_sensors::NortekDvl::parse_bytes(byte_msg, beam_angle, phased_array, max_clock_offset);

EXPECT_FALSE(ok);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(NortekDvlTest, validParses)
{
// Simple function to create a ds_sensors_msg::Dvl from arguments
  auto dvl = [](double x, double y, double z, double r0, double r1, double r2, double r3, double course,
                double speed) {

    auto msg = ds_sensor_msgs::Dvl{};

    // Velocities [m/s]
    msg.velocity.x = -x;
    msg.velocity.y = -y;
    msg.velocity.z = -z;

    msg.range[0] = r0;
    msg.range[1] = r1;
    msg.range[2] = r2;
    msg.range[3] = r3;

    msg.course_gnd = course;
    msg.speed_gnd = speed;

    for (int i = 0; i < 9; i++)
      msg.velocity_covar[i] = -1;

    msg.num_good_beams = 0;
    msg.speed_sound = 1500;

    msg.beam_unit_vec[0].x = 0.35355338;
    msg.beam_unit_vec[0].y = 0.35355338;
    msg.beam_unit_vec[0].z = 0.86602539;

    msg.beam_unit_vec[1].x = 0.35355338;
    msg.beam_unit_vec[1].y =-0.35355338;
    msg.beam_unit_vec[1].z = 0.86602539;

    msg.beam_unit_vec[2].x =-0.35355338;
    msg.beam_unit_vec[2].y =-0.35355338;
    msg.beam_unit_vec[2].z = 0.86602539;

    msg.beam_unit_vec[3].x =-0.35355338;
    msg.beam_unit_vec[3].y = 0.35355338;
    msg.beam_unit_vec[3].z = 0.86602539;


    for (int i = 0; i < 4; i++)
    {
      msg.raw_velocity_covar[i] = -1;
      msg.range_covar[i] = -1;
    }

    return msg;
  };

  auto df21 = [](uint8_t offsetOfData, uint32_t serialNumber, uint8_t day, uint8_t seconds,
                 uint32_t status) {

    auto msg_df21 = ds_sensor_msgs::NortekDF21{};
    // BOTTOM TRACK FIELDS- most important!
    msg_df21.version = 1;
    msg_df21.offsetOfData = offsetOfData;
    msg_df21.serialNumber = serialNumber;
    msg_df21.year = 119;
    msg_df21.month = 7;
    msg_df21.day = day;
    msg_df21.hour = 14;
    msg_df21.minute = 49;
    msg_df21.seconds = seconds;
    msg_df21.microSeconds = 10;
    msg_df21.nBeams = 0;
    msg_df21.error = 0;
    msg_df21.status = status;
    msg_df21.speed_sound = 1500.0;
    msg_df21.temperature = 21.964294;
    msg_df21.pressure= 0.057500001;
    msg_df21.velBeam = { 0, 0, 0, 0 };
    msg_df21.distBeam = { 0.0, 0.0, 0.0, 0.0 };
    msg_df21.fomBeam = { 10.0, 10.0, 10.0, 10.0 };
    msg_df21.timeDiff1Beam = {0.021577001, 0.021577001, 0.021577001, 0.021577001};
    msg_df21.timeDiff2Beam = {-0.134923, -0.134923, -0.134923, -0.134923};
    msg_df21.timeVelEstBeam = {0.0, 0.0, 0.0, 0.0};
    msg_df21.velX = -32.768;
    msg_df21.velY = -32.768;
    msg_df21.velZ1 = -32.768;
    msg_df21.velZ2 = -32.768;
    msg_df21.fomX = 10.0;
    msg_df21.fomY = 10.0;
    msg_df21.fomZ1 = 10.0;
    msg_df21.fomZ2 = 10.0;
    msg_df21.timeDiff1X = 0.0214859992266;
    msg_df21.timeDiff1Y = 0.0214859992266;
    msg_df21.timeDiff1Z1 = 0.0214859992266;
    msg_df21.timeDiff2Z2 = 0.0214859992266;
    msg_df21.timeDiff2X = -0.133916005492;
    msg_df21.timeDiff2Y = -0.133916005492;
    msg_df21.timeDiff2Z1 = -0.133916005492;
    msg_df21.timeDiff2Z2 = -0.133916005492;
    msg_df21.timeVelEstX = 0.0;
    msg_df21.timeVelEstY = 0.0;
    msg_df21.timeVelEstZ1 = 0.0;
    msg_df21.timeVelEstZ2 = 0.0;

    return msg_df21;
  };

  auto test_msg = dvl(32.768002,  32.768002,  32.768002, 0, 0, 0 ,0, -135, 46.340954);
  auto test_df21 = df21(36, 100996, 8, 21, 536870912);

  auto ok = false;
  auto parsed_msg = ds_sensor_msgs::Dvl{};
  auto parsed_df21 = ds_sensor_msgs::NortekDF21{};
  auto parsed_rng = ds_sensor_msgs::Ranges3D{};
  auto byte_msg = buildRaw(nortekdvl_test_data::pass, 0);

  std::tie(ok, parsed_df21) = ds_sensors::NortekDvl::parse_bytes(byte_msg, beam_angle, phased_array, max_clock_offset);
  ds_sensors::NortekDvl::msg_to_dvl(&parsed_msg, &parsed_df21, beam_angle, phased_array);

// Should have succeeded
  EXPECT_TRUE(ok);

// DVL ENTRIES -> lower priority but still important.
// All fields should match.
  EXPECT_FLOAT_EQ(test_msg.velocity.x, parsed_msg.velocity.x);
  EXPECT_FLOAT_EQ(test_msg.velocity.y, parsed_msg.velocity.y);
  EXPECT_FLOAT_EQ(test_msg.velocity.z, parsed_msg.velocity.z);

  for (int i = 0; i < 9; i++)
  {
    EXPECT_FLOAT_EQ(test_msg.velocity_covar[0], parsed_msg.velocity_covar[i]);
  }
  EXPECT_FLOAT_EQ(test_msg.speed_sound, parsed_msg.speed_sound);
  EXPECT_FLOAT_EQ(test_msg.num_good_beams, parsed_msg.num_good_beams);
  for (int i = 0; i < 4; i++)
  {
    EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].x, parsed_msg.beam_unit_vec[i].x);
    EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].y, parsed_msg.beam_unit_vec[i].y);
    EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].z, parsed_msg.beam_unit_vec[i].z);
    EXPECT_FLOAT_EQ(test_msg.range[i], parsed_msg.range[i]);
    EXPECT_FLOAT_EQ(test_msg.range_covar[i], parsed_msg.range_covar[i]);
    EXPECT_FLOAT_EQ(test_msg.raw_velocity_covar[i], parsed_msg.raw_velocity_covar[i]);
  }

  EXPECT_FLOAT_EQ(test_msg.course_gnd, parsed_msg.course_gnd);
  EXPECT_FLOAT_EQ(test_msg.speed_gnd, parsed_msg.speed_gnd);

// BOTTOM TRACK MATCHING
  EXPECT_FLOAT_EQ(test_df21.version, parsed_df21.version);
  EXPECT_FLOAT_EQ(test_df21.offsetOfData, parsed_df21.offsetOfData);
  EXPECT_FLOAT_EQ(test_df21.serialNumber, parsed_df21.serialNumber);
  EXPECT_FLOAT_EQ(test_df21.year, parsed_df21.year);
  EXPECT_FLOAT_EQ(test_df21.month, parsed_df21.month);
  EXPECT_FLOAT_EQ(test_df21.day, parsed_df21.day);
  EXPECT_FLOAT_EQ(test_df21.hour, parsed_df21.hour);
  EXPECT_FLOAT_EQ(test_df21.minute, parsed_df21.minute);
  EXPECT_FLOAT_EQ(test_df21.seconds, parsed_df21.seconds);
  EXPECT_FLOAT_EQ(test_df21.microSeconds, parsed_df21.microSeconds);
  EXPECT_FLOAT_EQ(test_df21.nBeams, parsed_df21.nBeams);
  EXPECT_FLOAT_EQ(test_df21.error, parsed_df21.error);
  EXPECT_FLOAT_EQ(test_df21.status, parsed_df21.status);
  EXPECT_FLOAT_EQ(test_df21.speed_sound, parsed_msg.speed_sound);
  EXPECT_FLOAT_EQ(test_df21.temperature, parsed_df21.temperature);
  EXPECT_FLOAT_EQ(test_df21.pressure, parsed_df21.pressure);
  EXPECT_FLOAT_EQ(test_df21.velX, parsed_df21.velX);
  EXPECT_FLOAT_EQ(test_df21.velY, parsed_df21.velY);
  EXPECT_FLOAT_EQ(test_df21.velZ1, parsed_df21.velZ1);
  EXPECT_FLOAT_EQ(test_df21.velZ2, parsed_df21.velZ2);

  for (int j = 0; j < 4; j++)
  {
    EXPECT_FLOAT_EQ(test_df21.velBeam[j], parsed_df21.velBeam[j]);
    EXPECT_FLOAT_EQ(test_df21.distBeam[j], parsed_df21.distBeam[j]);
    EXPECT_FLOAT_EQ(test_df21.fomBeam[j], parsed_df21.fomBeam[j]);
    EXPECT_FLOAT_EQ(test_df21.timeDiff1Beam[j], parsed_df21.timeDiff1Beam[j]);
    EXPECT_FLOAT_EQ(test_df21.timeDiff2Beam[j], parsed_df21.timeDiff2Beam[j]);
    EXPECT_FLOAT_EQ(test_df21.timeVelEstBeam[j], parsed_df21.timeVelEstBeam[j]);
  }
}

TEST_F(NortekDvlTest, BadSoundVelocity)
{
  ROS_ERROR_STREAM("Bad Sound Velocity Test");

  const auto test_pairs = std::list<std::pair<int, bool>>{ { 0, true },
                                                           { 459, false }};

  for (const auto& test_pair : test_pairs)
  {
    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_df21 = ds_sensor_msgs::NortekDF21{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};

    auto byte_msg = buildRaw(nortekdvl_test_data::badsoundspeed, test_pair.first);
    std::tie(ok, parsed_df21) = ds_sensors::NortekDvl::parse_bytes(byte_msg, beam_angle, phased_array, max_clock_offset);

    // Should pass
    EXPECT_EQ(test_pair.second, ok);
    if (test_pair.second)
    {
      EXPECT_EQ(1499.5, parsed_df21.speed_sound);
    }
  }
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  //ros::init(argc, argv, "test_NortekDvlparsing");
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  //ros::shutdown();
  return ret;
}
