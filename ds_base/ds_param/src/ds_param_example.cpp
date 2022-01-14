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
// Created by ivaughn on 1/30/18.
//

#include "ds_param/ds_param_conn.h"
#include "ds_param/ds_param_guard.h"

#include <boost/optional/optional_io.hpp>

class ParamDemo
{
public:
  ParamDemo(ros::NodeHandle& _nh) : handle(_nh)
  {
    conn = ds_param::ParamConnection::create(handle);

    // Grab a copy of our STATIC parameters
    // Note that this is a standard ROS call
    handle.getParam(ros::this_node::getName() + "/other_node", other_node);
    handle.getParam(ros::this_node::getName() + "/start_idx", start);

    // connect to OUR shared enum
    param_enum = conn->connect<ds_param::EnumParam>(ros::this_node::getName() + "/test_enum_param", true);
    // setup our shared enum
    param_enum->addNamedValue("Option 1", 1);
    param_enum->addNamedValue("non-consecutive Option 7", 7);

    // connect to OUR shared parameters
    param_int = conn->connect<ds_param::IntParam>(ros::this_node::getName() + "/test_int_param", true);
    param_str = conn->connect<ds_param::StringParam>(ros::this_node::getName() + "/test_str_param", true);

    // connect to the OTHER NODE's shared parameters
    other_int = conn->connect<ds_param::IntParam>("/" + other_node + "/test_int_param", false);
    other_str = conn->connect<ds_param::StringParam>("/" + other_node + "/test_str_param", false);

    // setup two parameters we'll update atomically
    param_bool_atomic = conn->connect<ds_param::BoolParam>(ros::this_node::getName() + "/test_atomic_bool", false);
    param_int_atomic = conn->connect<ds_param::IntParam>(ros::this_node::getName() + "/test_atomic_int", false);

    // test prep
    idx = start;
  }

  void _callback(const ros::TimerEvent& evt)
  {
    std::stringstream ss;
    ss << "Idx " << start << " -> " << idx;

    other_int->Set(idx);
    other_str->Set(ss.str());

    std::string old_str;
    if (param_int->GetPrevious())
    {
      old_str = std::to_string(param_int->GetPrevious().get());
    }
    else
    {
      old_str = "NOT SET";
    }

    // these two will update simulatenously
    {
      // this will prevent updates from going out
      ds_param::ParamGuard lock(conn);

      param_bool_atomic->Set(!param_bool_atomic->Get());
      param_int_atomic->Set(idx);
      if (param_int->GetPrevious())
      {
        ROS_ERROR_STREAM("\tResetting PARAM INT: " << param_int->Get() << " to " << param_int->GetPrevious());
        param_int->Set(*(param_int->GetPrevious()));
        ROS_ERROR_STREAM("\tReset PARAM INT: " << param_int->Get());
      }
    }

    ROS_ERROR_STREAM(ros::this_node::getName()

                     << ": "
                     << "MY param: " << param_int->Get() << " --> \"" << param_str->Get() << "\" (was " << old_str
                     << ")"

                     << "    OTHER param: " << other_int->Get() << " --> \"" << other_str->Get() << "\"");
    idx++;
  }

  // Setup the callback demos
  void setupCallback()
  {
    conn->setCallback(boost::bind(&ParamDemo::_change_callback, this, _1));
  }

  void _change_callback(const ds_param::ParamConnection::ParamCollection& params)
  {
    for (auto iter = params.begin(); iter != params.end(); iter++)
    {
      if (*iter == param_int)
      {
        ROS_ERROR_STREAM("\t\t\tPARAM INT Changed from \"" << param_int->GetPrevious() << "\" to \"" << param_int->Get()
                                                           << "\"");
      }
    }
  }

protected:
  ros::NodeHandle& handle;
  ds_param::ParamConnection::Ptr conn;
  int idx;

  // OUR static paramters
  std::string other_node;
  int start;

  // OUR shared parameters
  ds_param::EnumParam::Ptr param_enum;
  ds_param::IntParam::Ptr param_int;
  ds_param::StringParam::Ptr param_str;

  ds_param::BoolParam::Ptr param_bool_atomic;
  ds_param::IntParam::Ptr param_int_atomic;

  // The OTHER NODE's shared parameters
  ds_param::IntParam::Ptr other_int;
  ds_param::StringParam::Ptr other_str;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_node_1");
  ros::NodeHandle nh;
  ParamDemo demo(nh);

  ros::Timer timer = nh.createTimer(ros::Duration(3.0), boost::bind(&ParamDemo::_callback, &demo, _1), false);

  bool use_callback;
  if (!nh.getParam(ros::this_node::getName() + "/use_callback", use_callback))
  {
    ROS_FATAL_STREAM("No variable for use_callback!");
    return -1;
  }

  if (use_callback)
  {
    demo.setupCallback();
  }

  // spins until shut down
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}