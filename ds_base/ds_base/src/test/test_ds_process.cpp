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
#include "ds_base/ds_process.h"

#include <gtest/gtest.h>

using namespace ds_base;

#if 0
class SurfaceJoystickControllerTest: public SurfaceJoystickController
{
 public:
  SurfaceJoystickControllerTest(): SurfaceJoystickController(){}
  ~SurfaceJoystickControllerTest() override = default;

  uint64_t type() const noexcept override {
    return 0;
  }

 protected:
  void setupTransferFunctions() override {

  }
};
#endif

class ProcessTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    process_ = std::unique_ptr<DsProcess>(new DsProcess);
  }

  std::unique_ptr<DsProcess> process_;
};

TEST_F(ProcessTest, clean_exit)
{
  process_->setup();
  process_.reset();
}

void empty_callback(ds_core_msgs::RawData)
{
}

/*
 * This test is super important, but also very broken.
 * As of 24 June 2019 it works fine, but segfaults on destruction somewhere deep in ROS.
 * This LOOKS like a ROS bug, but I've been unable to reproduce.
TEST_F(ProcessTest, multiple_asio_connections)
{
  process_->setup();

  // auto str = ros::names::resolve(ros::this_node::getName(), std::string{"connection1"});
  auto con1 = process_->addConnection("connection1", &empty_callback);

  // str = ros::names::resolve(ros::this_node::getName(), std::string{"connection2"});
  auto con2 = process_->addConnection("connection2", &empty_callback);
}
*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ds_process");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  // spinner.stop();
  ros::shutdown();
  ros::waitForShutdown();
  return ret;
};
