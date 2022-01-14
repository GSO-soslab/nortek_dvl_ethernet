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
#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_match_functions.h"
#include <gtest/gtest.h>
#include <ostream>
#include "boost/asio/streambuf.hpp"

class MatchFunctionsTest: public ::testing::Test
{
 protected:

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

};

TEST_F(MatchFunctionsTest, match_char)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  os << "Test matcher!\n";

  auto obj = match_char('\n');

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_header_pd0)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x04);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_header_pd0_fail)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x08);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found,false);
}

TEST_F(MatchFunctionsTest, match_header_pd0_0length)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_multi_header_length_pass) {

  std::vector<std::vector<unsigned char>> headers = {{0xaf, 0x02}, {0xae, 0x02}};
  std::vector<int> lengths = {5, 7};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
        , {0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, true);
  }
}

TEST_F(MatchFunctionsTest, match_multi_header_length_poorlydefined) {

  std::vector<std::vector<unsigned char>> headers = {{}};
  std::vector<int> lengths = {5};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAF, 0x02, 0x03, 0x04, 0x05}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, false);
  }
}

TEST_F(MatchFunctionsTest, match_multi_header_length_fail) {

  std::vector<std::vector<unsigned char>> headers = {{0xaf}, {0xae}};
  std::vector<int> lengths = {5, 7};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAD, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A} //wrong header
        , {0xAE}
        , {0xAF, 0x02, 0x03}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, false);
  }
}

TEST_F(MatchFunctionsTest, match_header_read_length_pass_rdidvl) {
  std::vector<unsigned char> header= {0x7F, 0x7F};
  int length_location = 2;
  int length_bytes = 2;
  bool msb_first = false;
  int length_add = 2;
  int max_length = 50;

  std::vector<std::vector<unsigned char>> myData =
      { {0x7F, 0x7F, 0x04, 0x00, 0x00, 0x00}
        , {0x7F, 0x7F, 0x05, 0x00, 0x00, 0x00, 0x00}
      };

  auto obj = match_header_read_length(header, length_location, length_bytes, msb_first, length_add, max_length);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, true);
  }
}

TEST_F(MatchFunctionsTest, match_header_read_length_pass_different) {
  std::vector<unsigned char> header= {0xAA, 0xAB, 0xAC, 0xAD};
  int length_location = 4;
  int length_bytes = 2;
  bool msb_first = true;
  int length_add = 0;
  int max_length = 500;

  std::vector<unsigned char> data(300, 0x00);
  data[0] = 0xAA;
  data[1] = 0xAB;
  data[2] = 0xAC;
  data[3] = 0xAD;
  data[4] = 0x01;
  data[5] = 0x2C;

  auto obj = match_header_read_length(header, length_location, length_bytes, msb_first, length_add, max_length);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  boost::asio::streambuf b;
  std::ostream os(&b);
  ROS_INFO_STREAM("Data "<< data.data());
  os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_header_read_length_pass_nortek_dvl) {
  std::vector<unsigned char> header= {0xA5};
  int length_location = 1;
  int length_bytes = 1;
  bool msb_first = true;
  int length_add = 0;
  int max_length = 50;

  std::vector<unsigned char> data(37, 0x00);
  data[0] = 0xA5;
  data[1] = 37;
  data[2] = 0x01;
  data[3] = 0x01;
  data[4] = 0x01;

  auto obj = match_header_read_length(header, length_location, length_bytes, msb_first, length_add, max_length);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  boost::asio::streambuf b;
  std::ostream os(&b);
  ROS_INFO_STREAM("Data "<< data.data());
  os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  ASSERT_EQ(found, true);
}



// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
