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
// Created by ivaughn on 1/10/18.
//
// I/O State Machine
//
// The IO State Machine is described more fully in IO_SM.md//
//

#ifndef PROJECT_DS_IOSM_H
#define PROJECT_DS_IOSM_H

#include <string>
#include <memory>
#include <mutex>
#include <list>
#include <utility>

#include <functional>

#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/noncopyable.hpp>

#include <ds_asio/ds_connection.h>
#include "ds_core_msgs/RawData.h"
#include "ds_core_msgs/IoCommand.h"
#include <ros/duration.h>

namespace ds_asio
{
///  @brief The IoCommand class is a single entry in the command table
///
///  IoCommands specify timeouts, delays, and acceptance criteria.  Each basic
///  command implements the following steps:
///
///  1. [optional] delay before sending a command
///  2. Send a provided string
///  3. Wait for an acceptable response.  More on this later
///  4. Create a new event based on that response
///  5. [optional] delay before moving on to the next command
///
class IoCommand
{
public:
  typedef std::shared_ptr<IoCommand> Ptr;
  typedef std::shared_ptr<const IoCommand> ConstPtr;

  typedef boost::function<bool(ds_core_msgs::RawData)> ReadCallback;
  typedef boost::function<void()> TimeoutCallback;

  /// @brief Shorthand to create a standard command/timeout pair
  ///
  /// \param cmdstr The command string to send
  /// \param timeout_sec  The number of seconds of timeout time for the command (NO other timeouts are applied!)
  /// \param _force_next Allow a non-regular-command after this one (OFF by default)
  /// \param callback A callback fired when the response to this particular command is received
  IoCommand(const std::string& cmdstr, double timeout_sec, bool _force_next = false,
            const ds_asio::IoCommand::ReadCallback& callback = ds_asio::IoCommand::ReadCallback());

  /// @brief Shorthand to create a static wait
  ///
  /// The primary use of this is to add a delay to slow down interrogation loops
  ///
  /// \param timeout_sec Number of seconds to wait
  IoCommand(double timeout_sec);

  /// @brief Fill in a command string using a IoCommand ROS message
  IoCommand(const ds_core_msgs::IoCommand& _cmd);

  /// @brief Set the command string to send
  void setCommand(const std::string& _cmd);

  /// @brief Get the command string that will be sent
  const std::string& getCommand() const;

  /// @brief Set the delay before the command string is sent
  void setDelayBefore(const ros::Duration& _d);

  /// @brief Get the delay before the command string is sent
  const ros::Duration& getDelayBefore() const;

  /// @brief Set the delay after a valid reply is received, but
  /// before the next command is started
  void setDelayAfter(const ros::Duration& _d);

  /// @brief Get the delay after a valid reply is received, but
  /// before the next command is started
  const ros::Duration& getDelayAfter() const;

  /// @brief Set the maximum time to wait for a valid reply
  void setTimeout(const ros::Duration& _t);

  /// @brief Get the maximum time to wait for a valid reply
  const ros::Duration& getTimeout() const;

  /// @brief Check whether this command will generate a callback on
  /// accepting an response
  bool emit() const;

  /// @brief Set whether this command will generate a callback on
  /// accepting an response
  void setEmit(bool _e);

  /// @brief Check whether this command will log a warning to the ROS console on
  /// timeout
  bool warnOnTimeout() const;

  /// @brief Set whether this command will log a warning to the ROS console on
  /// timeout
  void setWarnOnTimeout(bool _w);
  
  /// @brief Check whether this command will log an error to the ROS console on
  /// state transition guard condition
  bool errOnStateTrans() const;

  /// @brief Set whether this command will log an error to the ROS console on
  /// state transition guard condition
  void setErrOnStateTrans(bool _err);

  /// @brief Check whether a preempt command is allowed to follow this command
  ///
  /// Some commands require multiple inputs with return checking in between.  One
  /// key example is setting an address on a bus.  The "Force Next" option
  /// forces the next command to come from the same queue as this one.
  bool getForceNext() const;

  /// @brief Set whether a preempt command is allowed to follow this command
  ///
  /// Some commands require multiple inputs with return checking in between.  One
  /// key example is setting an address on a bus.  The "Force Next" option
  /// forces the next command to come from the same queue as this one.
  void setForceNext(bool _fn);

  /// @brief Get the state machine's ID for this command.  Used to override existing
  /// entries
  uint64_t getId() const;

  /// @brief Set the state machine's ID for this command.  Set by the state machine
  void setId(uint64_t);

  /// @brief Check to see if this command has a custom callback
  bool hasCallback() const;

  /// @brief Get the callback function for this command
  ///
  /// \return The function to call on receipt of data for this command
  const ReadCallback& getCallback() const;

  /// @brief Set the callback for this command
  ///
  /// \return The function to call on receipt of data for this command
  void setCallback(const ReadCallback& _cb);

  /// @brief Get the timeout callback for this command
  ///
  /// \return The function called when this command times out
  const TimeoutCallback& getTimeoutCallback() const;

  /// @brief Set the timeout callback called when this command times out
  void setTimeoutCallback(const TimeoutCallback& _cb);

  /// @brief Get the current size of the preempt queue
  size_t getPreemptQueueSize() const;
  
  bool stateTransErr;
protected:
  uint64_t id;
  std::string cmd;
  bool emitOnMatch; // whether to pass the response to the callbacks
                    // This is useful to prevent timeout-only IoCommands from
                    // calling the default callback.
  bool timeoutWarn; // emit ROS_WARN message if command times out
  bool forceNext;   // force the next command to be from the same queue
  //bool stateTransErr; // emit ROS_ERROR message if state transition false
  // This is useful in some cases where a command
  // sequence should happen atomically, e.g.,
  // we send an address command and then a
  // query or something
  ros::Duration delayBefore;
  ros::Duration delayAfter;
  ros::Duration timeout;

  ReadCallback callback;
  TimeoutCallback timeoutCallback;
};

// forward declaration
namespace iosm_inner
{
class _IoSM_impl;
};

/// @brief The public IoState Machine class.
///
/// This class has a fairly gnarly implementation, but the details are hidden behind
/// a pointer to the implementation.  In particular, note that
/// the IoSM class carries a shared_ptr to the actual data-- therefore, its
/// pretty easy to move around, etc.
class IoSM
{
public:
  IoSM(boost::asio::io_service& io_service, std::string name, const ds_asio::IoCommand::ReadCallback& callback);

  IoSM(boost::asio::io_service& io_service, std::string name);

  /// @brief Destructor.
  ///
  /// If there are still pending operations this may not immediately stop
  /// them.
  virtual ~IoSM();

  /// @brief Set the I/O connection.
  ///
  /// Necessary because we have to have our IoSM to setup the callback for the
  /// I/O Connection before we create the thing.
  void setConnection(const boost::shared_ptr<DsConnection>& conn);

  /// @brief Get the I/O Connection
  const boost::shared_ptr<DsConnection>& getConnection() const;

  /// @brief Set the IoSM-wide callback
  void setCallback(const ds_asio::IoCommand::ReadCallback& cb);

  /// @brief Get a copy of the IoSM-wide callback that fires every time this
  /// object is called
  const ds_asio::IoCommand::ReadCallback& getCallback() const;

  /// @brief Add a regular command to the regular command queue
  ///
  /// \param cmd The command to run
  /// \return The unique ID for this command
  uint64_t addRegularCommand(const IoCommand& cmd);

  /// @brief Delete a regular command from the queue
  ///
  /// \param id The ID of the command to delete (returned by addRegularCommand)
  void deleteRegularCommand(const uint64_t id);

  /// @brief Overwrite an existing command in the regular queue
  ///
  /// \param id The ID of the command to overwrite
  /// \param cmd The new command to execute instead
  bool overwriteRegularCommand(const uint64_t id, const IoCommand& cmd);

  /// @brief Add a command to the preempt queue.  This command will be run
  /// as soon as the bus is available
  ///
  /// \param cmd The new command to run ASAP.
  void addPreemptCommand(const IoCommand& cmd);

  /// @brief Access the list of regular commands.
  ///
  /// Provides read-only access to the list of commands.
  ///
  /// \return A copy of the list of regular commands
  std::list<ds_asio::IoCommand> getRegularCommands() const;

  /// @brief Access the number of commands in the preempt queue.  Note that this can decrease
  /// without warning as commands are executed, but the current value is typically a good
  /// upper-bound.  For bonus points, access while locking the Queue structures
  size_t getPreemptQueueSize() const;

protected:
  /// \brief Shared pointer to our implementation
  std::shared_ptr<ds_asio::iosm_inner::_IoSM_impl> impl;
};

}  // namespace ds_asio

#endif  // PROJECT_DS_IOSM_H
