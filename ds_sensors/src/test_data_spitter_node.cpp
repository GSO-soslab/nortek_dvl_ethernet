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
#include "test_data_spitter.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

int main(int argc, char* argv[])
{
  auto test_node = std::unique_ptr<TestDataSpitter>(new TestDataSpitter(argc, argv, "test_data_spitter"));

  auto test_nh = test_node->nodeHandle();

  auto update_rate = int{ 0 };
  auto test_string = boost::shared_ptr<std::string>(new std::string);

  if (!test_nh.getParam(ros::this_node::getName() + "/update_rate", update_rate))
  {
    ROS_FATAL("Unable to load update_rate parameter");
  }

  if (!test_nh.getParam(ros::this_node::getName() + "/test_data", *test_string))
  {
    ROS_FATAL("Unable to load test_data parameter.");
  }

  auto split = ros::param::param<bool>("~split", true);

  boost::algorithm::replace_all(*test_string, "\\r", "\r");
  boost::algorithm::replace_all(*test_string, "\\n", "\n");

  auto messages = std::list<std::string>{};

  boost::algorithm::split_regex(messages, *test_string, boost::regex("\r\n"));
  // split can return empty strings, get rid of those.
  messages.erase(std::remove_if(std::begin(messages), std::end(messages), [](std::string& s) { return s.empty(); }),
                 std::end(messages));
  test_node->setup();
  auto connection = test_node->connection("instrument");
  // ROS_ASSERT(connection);

  auto timer = test_nh.createTimer(ros::Duration(1.0 / update_rate), [&](const ros::TimerEvent&) {
    if (split)
    {
      for (auto& m : messages)
      {
        auto msg = boost::shared_ptr<std::string>{ new std::string{ m } };
        ROS_INFO_STREAM("Sending:  " << *msg);
        connection->send(std::move(msg));
      }
    }
    else
    {
      connection->send(test_string);
    }
  });

  test_node->run();
}
