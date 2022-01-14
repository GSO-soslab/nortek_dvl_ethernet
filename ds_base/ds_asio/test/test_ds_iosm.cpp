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
// Created by ivaughn on 1/19/18.
//

#include "ds_asio/ds_iosm.h"
#include "ds_asio/ds_mock_connection.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <list>
#include <gtest/gtest.h>


class ds_iosm_test : public ::testing::Test {

protected:
    /// \brief Setup a new I/O state machine and mock connection
    virtual void SetUp() {
        callback_default_return = true;
        timeout_callbacks = 0;

        // initialize ROS
        ros::Time::init();

        // setup our dummy connection
        conn.reset(new ds_asio::DsMockConnection(io_service));

        // connect our state machine to it
        iosm.reset(new ds_asio::IoSM(io_service, "iosm_under_test"));
        iosm->setConnection(conn);
        iosm->setCallback(boost::bind(&ds_iosm_test::_iosm_callback, this, _1));
    }

    // Run through all the tests until we're done
    virtual void runConnection() {

        start  = boost::posix_time::microsec_clock::universal_time();
        conn->run();
        finish = boost::posix_time::microsec_clock::universal_time();
        runtime = finish - start;
    }

    // Helper function to build a raw data message from a string
    ds_core_msgs::RawData buildRawMsg(const std::string& msg) {
        ds_core_msgs::RawData toSend;
        toSend.header.stamp = ros::Time::now();
        toSend.ds_header.io_time = toSend.header.stamp;
        toSend.data_direction = ds_core_msgs::RawData::DATA_IN;
        toSend.data = std::vector<uint8_t>(msg.begin(), msg.end());
        return toSend;
    }

    bool _iosm_callback(ds_core_msgs::RawData data) {
        received_data.push_back(data);
        bool rc;
        if (!callback_return_codes.empty()) {
            rc = callback_return_codes.front();
            callback_return_codes.pop_front();
        } else {
            rc = callback_default_return;
        }
        return rc;
    }

 public:
    void _iosm_timeout_callback() {
        timeout_callbacks++;
    }

 protected:
    std::string receivedString(size_t idx) {
        const ds_core_msgs::RawData& msg = received_data[idx];
        return std::string(msg.data.begin(), msg.data.end());
    }

    boost::shared_ptr<ds_asio::IoSM> iosm;
    boost::shared_ptr<ds_asio::DsMockConnection> conn;
    boost::asio::io_service io_service;

    // Timing data
    boost::posix_time::ptime start;
    boost::posix_time::ptime finish;
    boost::posix_time::time_duration runtime;

    // a list of data fired from callbacks
    std::deque<ds_core_msgs::RawData> received_data;
    std::deque<bool> callback_return_codes;
    bool callback_default_return;

    int timeout_callbacks;

    //virtual void TearDown() {}
};

TEST_F(ds_iosm_test, basic_send) {

    // The standard test is
    iosm->addRegularCommand(ds_asio::IoCommand("test_query", 0.01));

    runConnection();

    ASSERT_EQ(1, conn->Written().size());
    EXPECT_EQ(std::string("test_query"), conn->Written()[0]);
}

TEST_F(ds_iosm_test, reply_test) {

    // The standard test is
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_1", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));

    ds_core_msgs::RawData toSend = buildRawMsg("test_reply_1");
    conn->ToRead().push_back(toSend);

    runConnection();

    // check the data that went out
    ASSERT_EQ(2, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);

    // check data that passed through the I/O state machine
    ASSERT_EQ(1, received_data.size());
    EXPECT_EQ("test_reply_1", receivedString(0));

    // check that the total test time was reasonably speedy
    EXPECT_GE(5, runtime.total_milliseconds());
}

TEST_F(ds_iosm_test, wrap_test) {

    // The standard test is
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_1", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));

    conn->ToRead().push_back(buildRawMsg("test_reply_1"));
    conn->ToRead().push_back(buildRawMsg("test_reply_2"));
    conn->ToRead().push_back(buildRawMsg("test_reply_3"));
    conn->ToRead().push_back(buildRawMsg("test_reply_4"));

    runConnection();

    // check the data that went out
    ASSERT_EQ(5, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[2]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[3]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[4]);

    // check data that passed through the I/O state machine
    ASSERT_EQ(4, received_data.size());
    EXPECT_EQ("test_reply_1", receivedString(0));
    EXPECT_EQ("test_reply_2", receivedString(1));
    EXPECT_EQ("test_reply_3", receivedString(2));
    EXPECT_EQ("test_reply_4", receivedString(3));

    // check that the total test time was reasonably speedy
    EXPECT_GE(5, runtime.total_milliseconds());
}

TEST_F(ds_iosm_test, timeout_test) {

    // The standard test is
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_1", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));

    // First: no response
    ds_core_msgs::RawData toSend = buildRawMsg("");
    conn->ToRead().push_back(toSend);

    // Next: a response (the I/O state machine MUST end on a response)
    toSend = buildRawMsg("test_reply");
    conn->ToRead().push_back(toSend);

    runConnection();

    // we actually get an extra query after the end
    ASSERT_EQ(3, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[2]);
    EXPECT_LE( 99, runtime.total_milliseconds());
    EXPECT_GE(105, runtime.total_milliseconds());
}

TEST_F(ds_iosm_test, timeout_callback_test) {

    // The standard test is
    ds_asio::IoCommand query1("test_query_1", 0.10);
    query1.setTimeoutCallback(boost::bind(&ds_iosm_test::_iosm_timeout_callback, this));
    iosm->addRegularCommand(query1);
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));

    // First: no response
    ds_core_msgs::RawData toSend = buildRawMsg("");
    conn->ToRead().push_back(toSend);

    // Next: a response (the I/O state machine MUST end on a response)
    toSend = buildRawMsg("test_reply");
    conn->ToRead().push_back(toSend);

    runConnection();

    // we actually get an extra query after the end
    ASSERT_EQ(3, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[2]);
    EXPECT_LE( 99, runtime.total_milliseconds());
    EXPECT_GE(105, runtime.total_milliseconds());
    EXPECT_EQ(1, timeout_callbacks);
}

TEST_F(ds_iosm_test, reject_str) {

    // we'll create 3 commands
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_1", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_3", 0.10));

    // and 3 incoming data messages
    conn->ToRead().push_back(buildRawMsg("test_reply_1"));
    conn->ToRead().push_back(buildRawMsg("test_reply_2"));
    conn->ToRead().push_back(buildRawMsg("test_reply_3"));

    // we'll setup up the callback to reject the second reply...
    callback_return_codes.push_back(true);
    callback_return_codes.push_back(false);
    callback_return_codes.push_back(true);

    // The connection will run until we've sent test_reply_1,2 and 3.
    // However, having rejected test_reply_2 the iosm shouldn't ever
    // send test_query_3
    runConnection();

    // I'd really PREFER to not see test_query_3 written, but the mock class
    // can only send a single reply to a single written message.  This
    // forces us to look for the timeout instead.  Not idea, but it's fine.
    ASSERT_EQ(4, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[2]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[3]);
    EXPECT_LE( 99, runtime.total_milliseconds());
    EXPECT_GE(105, runtime.total_milliseconds());
}

TEST_F(ds_iosm_test, pre_delay) {

    // The standard test is
    ds_asio::IoCommand delay_cmd("test_query_1", 0.20);
    delay_cmd.setDelayBefore(ros::Duration(0.10));
    iosm->addRegularCommand(delay_cmd);
    // not all of these will get run
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_3", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_4", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_5", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_6", 0.20));

    // setup a couple replies
    conn->ToRead().push_back(buildRawMsg("test_reply_1"));
    conn->ToRead().push_back(buildRawMsg("test_reply_2"));
    conn->ToRead().push_back(buildRawMsg("test_reply_3"));
    conn->ToRead().push_back(buildRawMsg("test_reply_4"));

    runConnection();

    // we actually get an extra query after the end
    ASSERT_LE(3, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[2]);

    ASSERT_LE(2, received_data.size());
    EXPECT_EQ("test_reply_1", receivedString(0));
    EXPECT_EQ("test_reply_2", receivedString(1));

    EXPECT_LE( 99, runtime.total_milliseconds());
    EXPECT_GE(105, runtime.total_milliseconds());

    boost::posix_time::time_duration start_to_reply1  = received_data[0].header.stamp.toBoost() - start;
    boost::posix_time::time_duration reply1_to_finish = finish - received_data[0].header.stamp.toBoost();

    EXPECT_LE( 99, start_to_reply1.total_milliseconds());
    EXPECT_GE(105, start_to_reply1.total_milliseconds());

    EXPECT_GE(  5, reply1_to_finish.total_milliseconds());
}

TEST_F(ds_iosm_test, post_delay) {

    // The standard test is
    ds_asio::IoCommand delay_cmd("test_query_1", 0.20);
    delay_cmd.setDelayAfter(ros::Duration(0.10));
    iosm->addRegularCommand(delay_cmd);
    // not all fo these will get run
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_3", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_4", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_5", 0.20));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_6", 0.20));

    // setup a couple replies
    conn->ToRead().push_back(buildRawMsg("test_reply_1"));
    conn->ToRead().push_back(buildRawMsg("test_reply_2"));
    conn->ToRead().push_back(buildRawMsg("test_reply_3"));
    conn->ToRead().push_back(buildRawMsg("test_reply_4"));

    runConnection();

    // we actually get an extra query after the end
    ASSERT_LE(3, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[2]);

    ASSERT_LE(2, received_data.size());
    EXPECT_EQ("test_reply_1", receivedString(0));
    EXPECT_EQ("test_reply_2", receivedString(1));

    EXPECT_LE( 99, runtime.total_milliseconds());
    EXPECT_GE(105, runtime.total_milliseconds());

    boost::posix_time::time_duration start_to_reply1  = received_data[0].header.stamp.toBoost() - start;
    boost::posix_time::time_duration reply1_to_finish = finish - received_data[0].header.stamp.toBoost();

    EXPECT_GE(  5, start_to_reply1.total_milliseconds());

    EXPECT_LE( 99, reply1_to_finish.total_milliseconds());
    EXPECT_GE(105, reply1_to_finish.total_milliseconds());
}

TEST_F(ds_iosm_test, delete_test) {

    // The standard test is
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_1", 0.10));
    uint64_t cmd_id = iosm->addRegularCommand(ds_asio::IoCommand("test_query_2", 0.10));
    iosm->addRegularCommand(ds_asio::IoCommand("test_query_3", 0.10));

    conn->ToRead().push_back(buildRawMsg("test_reply_1"));
    conn->ToRead().push_back(buildRawMsg("test_reply_2"));
    conn->ToRead().push_back(buildRawMsg("test_reply_3"));
    conn->ToRead().push_back(buildRawMsg("test_reply_4"));

    runConnection();

    // check the data that went out
    ASSERT_EQ(5, conn->Written().size());
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[0]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[1]);
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[2]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[3]);
    EXPECT_EQ(std::string("test_query_2"), conn->Written()[4]);

    // check data that passed through the I/O state machine
    ASSERT_EQ(4, received_data.size());
    EXPECT_EQ("test_reply_1", receivedString(0));
    EXPECT_EQ("test_reply_2", receivedString(1));
    EXPECT_EQ("test_reply_3", receivedString(2));
    EXPECT_EQ("test_reply_4", receivedString(3));

    // check that the total test time was reasonably speedy
    EXPECT_GE(5, runtime.total_milliseconds());

    // Ok, now that we're good so far... go ahead and delete command #2
    iosm->deleteRegularCommand(cmd_id);

    conn->setWriteDuringStartup();

    // continue the test
    conn->ToRead().push_back(buildRawMsg("test_reply_5"));
    conn->ToRead().push_back(buildRawMsg("test_reply_6"));
    conn->ToRead().push_back(buildRawMsg("test_reply_7"));
    conn->ToRead().push_back(buildRawMsg("test_reply_8"));

    // actually run the test
    conn->run();

    // start looking for data sent by the state machine; note that
    // cmd 2 has been deleted!
    ASSERT_EQ(9, conn->Written().size());
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[5]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[6]);
    EXPECT_EQ(std::string("test_query_3"), conn->Written()[7]);
    EXPECT_EQ(std::string("test_query_1"), conn->Written()[8]);

    ASSERT_EQ(8, received_data.size());
    EXPECT_EQ("test_reply_5", receivedString(4));
    EXPECT_EQ("test_reply_6", receivedString(5));
    EXPECT_EQ("test_reply_7", receivedString(6));
    EXPECT_EQ("test_reply_8", receivedString(7));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}