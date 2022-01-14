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

#ifndef PROJECT_DS_IOSM_PRIVATE_H
#define PROJECT_DS_IOSM_PRIVATE_H

#include <ds_asio/ds_iosm.h>
#include <ds_asio/ds_connection.h>
#include <ros/console.h>

#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace ds_asio
{
namespace iosm_inner
{
class Runner_front;
typedef boost::msm::back::state_machine<Runner_front> Runner;

// ////////////////////////////////////////////////////////////////////////////////////
/// @brief Implementation of an IO State Machine.  Handles callbacks, etc
class _IoSM_impl : public std::enable_shared_from_this<_IoSM_impl>
{
private:
  // make non-copyable by declaring the constructor, copy constructor, and assignment
  // operators private

  /// @brief Standard constructor.  You want the create method.
  ///
  /// for shared_from_this to work correctly, we need to ensure ONLY pointers
  /// to shared objects are created
  _IoSM_impl(boost::asio::io_service& io_service, std::string name,
             const boost::function<bool(ds_core_msgs::RawData)>& callback);

  // no implementation of these is required
  // (private copy constructor / operators are a special thing)
  _IoSM_impl(const _IoSM_impl& other);       // non-copy-constructable
  _IoSM_impl& operator=(const _IoSM_impl&);  // non-copyable

public:
  typedef std::shared_ptr<ds_asio::iosm_inner::_IoSM_impl> Ptr;
  typedef std::weak_ptr<ds_asio::iosm_inner::_IoSM_impl> WeakPtr;
  typedef std::shared_ptr<const ds_asio::iosm_inner::_IoSM_impl> ConstPtr;

  /// @brief Method to create this class INSTEAD of the standard constructor
  ///
  /// The I/O state machine MUST always be instantiated as an std::shared_ptr.
  /// The correct way to create this class is using the create static method
  /// that will set up the shared pointer for you.
  /// Creating an _IoSM_impl as anything other than a shared pointer risks
  /// leaving a dangling pointer in the asio infrastructure that could
  /// cause a segfault.  This pattern lets us use a shared_ptr that will
  /// not be deallocated until everything is done using the object.
  ///
  /// \return A new instance of this class
  static std::shared_ptr<_IoSM_impl> create(boost::asio::io_service& io_service, std::string name,
                                            const boost::function<bool(ds_core_msgs::RawData)>& callback);

  /// @brief Destructor
  virtual ~_IoSM_impl();

  /// @brief Set the I/O connection.
  ///
  /// Necessary because we have to have our IoSM to setup the callback for the
  /// I/O Connection before we create the thing.
  void setConnection(const boost::shared_ptr<ds_asio::DsConnection>& conn);

  /// @brief Set the IoSM-wide callback
  void setCallback(const ds_asio::IoCommand::ReadCallback& cb);

  /// @brief Get a copy of the IoSM-wide callback that fires every time this
  /// object is called
  const ds_asio::IoCommand::ReadCallback& getCallback() const;

  /// @brief
  ///
  /// Runner can't be initialized during constructor time
  void start();

  /// @brief Stop all pending ops and start to shutdown the state machine.
  void shutdown();

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

  // TODO: Shutdown queue?

  /// @brief Accessor for the list of regular commands
  std::list<ds_asio::IoCommand> getRegularCommands();

  /// @brief Get the length of the regular commands
  size_t getPreemptQueueSize();

  const boost::shared_ptr<DsConnection>& getConnection() const;

protected:

  /// \brief Mutex to protect the command queues.
  ///
  /// Should be locked before accessing nextCmdId, regularCommands, currCommand,
  /// preemptCommands, commandRunning, isPreemptCommand.
  /// This is always the inner mutex.
  std::mutex _queueMutex;

  /// \brief Mutex to protect calls to process_event
  ///
  /// boost::msm's process_event is re-entrant but not thread safe.
  /// (https://lists.boost.org/Archives/boost/2010/08/170260.php)
  /// We want events to be able to generate additional events, while
  /// properly serializing events from other threads (e.g. ROS callbacks).
  std::recursive_mutex _processEventMutex;

  /// \brief ID that will be assigned to the next regular command that is added.
  ///
  /// Guarantees uniqueness by only incrementing.
  uint64_t nextCmdId;

  /// \brief The list of regular commands
  ///
  /// Regular commands are added to the queue in the order they come in.
  /// When the regular command list has been run completely, it restarts
  /// from the top.
  std::list<ds_asio::IoCommand> regularCommands;

  /// \brief The next command in the regular queue to run.
  ///
  /// An std::list iterator is pretty much just a pointer, so we use that.
  std::list<ds_asio::IoCommand>::iterator currCommand;

  /// \brief The Preempt Queue. Commands executed in the order they are received
  ///
  /// Preempt commands
  std::list<ds_asio::IoCommand> preemptCommands;

  /// \brief IoService reference (from parent)
  boost::asio::io_service& io_service_;

  /// \brief Connection to the bus physical layer via ds_asio
  boost::shared_ptr<ds_asio::DsConnection> connection_;

  /// \brief Deadline timer used to implement timeouts, etc
  boost::asio::deadline_timer timeoutTimer_;

  /// \brief IoSM name
  std::string name_;

  /// \brief Callback fired when a message is ready to go out
  ds_asio::IoCommand::ReadCallback callback_;

  /// \brief The MSM state machine that runs a single command
  std::shared_ptr<Runner> runner;

  /// \brief A flag to track whether a command is currently running
  bool commandRunning;

  /// \brief A flag to track if the command currently running is a preempt command or not
  bool isPreemptCommand;

  /// \friend Let the Runner_front poke at the following protected methods.
  /// Yes, it also allows access to things we probably shouldn't mess with, but
  /// the danger of someone else calling the _nolock methods is WAY worse.
  friend Runner_front;

  bool _dataReady_nolock(const ds_asio::IoCommand& cmd, const ds_core_msgs::RawData& raw);
  void _callTimeoutCallback_nolock(const ds_asio::IoCommand& cmd);
  void _sendData_nolock(const std::string& data);
  void _setTimeout_nolock(const ros::Duration& timeout);
  void _cancelTimeout_nolock();
  void _commandDone();
  void _runNextCommand();

  // callbacks called by external processes
public:
  void _timeoutCallback(const boost::system::error_code& error);  // fired when a timer expires
  void _dataCallback(const ds_core_msgs::RawData& raw);
};

// ////////////////////////////////////////////////////////////////////////////////////
// State Machine Events
/// @brief Event generated when a timer expires (used mostly for timeouts)
struct TimerDone
{
  TimerDone()
  {
  }
};

/// @brief Event that is fired when data is received
struct DataRecv
{
  DataRecv(const ds_core_msgs::RawData& _m) : msg(_m){};

  /// @brief Message received
  const ds_core_msgs::RawData& msg;
};

/// @brief Event that starts running a command (provided)
struct StartCommand
{
  StartCommand(const ds_asio::IoCommand& _cmd) : cmd(_cmd){};
  /// @brief The command to run
  const ds_asio::IoCommand cmd;
};

// ////////////////////////////////////////////////////////////////////////////////////
/// @brief Runs a single command
///
/// This class uses a boost::msm state machine to manage the execution of
/// a single command.  The state machine looks like this:
///
////    +-----------+
///    | Pre Delay |   (optional) delay before executing command
///    +-----------+
///         |
///         |    Delay timer expires
///        \ /
///    +-----------------+
///    | Accumulate Data |------+
///    +-----------------+      |
///         |                   | Data received matches (optional) regex
///         |Timeout expires    | (NOTE: Emits a signal with matched data)
///        \ /                  |
///    +------------+<----------+
///    | Post Delay |   (optional) delay after executing command
///    +------------+
///         |
///         |    Delay timer expires
///        \ /
///    +-------------+
///    | Done / Trap |
///    +-------------+

struct Runner_front : public msm::front::state_machine_def<Runner_front>
{
  /// @brief A pointer back to the state machine runner's
  /// various handy members.  We really can't store them in this class,
  /// or compilation won't work at all.
  ///
  /// This creates a circular reference, but guarantees that the
  /// state machine exists as long as CommandRunner_ does.
  /// The circular reference means that the _IoSM_Impl won't be destroyed
  /// normally by simply removing references to it.  The solution is to
  /// have an outer IoSM object that contains an _IoSM_Impl that has responsibility
  /// for telling _IoSM_Impl to break its circular reference
  /// and allow deallocation to happen

  IoCommand cmd;
  _IoSM_impl::Ptr sm;

  Runner_front(const std::shared_ptr<_IoSM_impl>& _sm) : cmd(1.0), sm(_sm)
  {
    // do nothing
  }

  /// @brief Reset our pointer to the parent StateMachine class.
  ///
  /// The only way to break the circular reference is for the _IoSM_impl owning
  /// this object to do so manually.
  void invalidateSm()
  {
    sm.reset();
  }

  /// @brief The "Ready to start a new command" state
  struct Ready : public msm::front::state<>
  {
    /// @brief Function called on entering the "ready" state
    ///
    /// \tparam Event Event Type (handled automagically)
    /// \tparam FSM Finite state machine type (some derivative of Runner_, handled automagically)
    /// \param evt The event that put us in this state
    /// \param fsm The command runner
    template <class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.sm)
      {
        fsm.sm->_commandDone();
      }
      else
      {
        ROS_WARN_STREAM("Ready::on_entry not calling command done; may result in a hung state machine");
      }
    }
  };

  /// @brief The "Delaying before sending data" state
  struct PreDelay : public msm::front::state<>
  {
    /// @brief Function called on entering the "wait to send data" state
    ///
    /// Setups up the timeout to fire when appropriate
    ///
    /// \tparam Event Event Type (handled automagically)
    /// \tparam FSM Finite state machine type (some derivative of Runner_, handled automagically)
    /// \param evt The event that put us in this state
    /// \param fsm The command runner
    template <class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.sm)
      {
        fsm.sm->_setTimeout_nolock(fsm.cmd.getDelayBefore());
      }
    }
  };

  /// @brief The "Wait for data" state- automatically sends command upon entering state
  struct WaitForData : public msm::front::state<>
  {
    /// @brief Function called on entering the "Wait for reply" state.  Sends command
    ///
    /// Actually sends the data to the connection
    ///
    /// \tparam Event Event Type (handled automagically)
    /// \tparam FSM Finite state machine type (some derivative of Runner_, handled automagically)
    /// \param evt The event that put us in this state
    /// \param fsm The command runner
    template <class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      // First, send the command
      ROS_DEBUG_STREAM("State machine sending command: " << fsm.cmd.getCommand());

      if (fsm.sm && !fsm.cmd.getCommand().empty())
      {
        fsm.sm->_sendData_nolock(fsm.cmd.getCommand());
      }

      // setup our timeout
      if (fsm.sm && fsm.cmd.getTimeout() > ros::Duration(0))
      {
        fsm.sm->_setTimeout_nolock(fsm.cmd.getTimeout());
      }
    }

    /// @brief Function called when a successful reply has us leaving the WaitForData state
    ///
    /// Cleans up any lingering timeouts, etc
    ///
    /// \tparam Event Event Type (handled automagically)
    /// \tparam FSM Finite state machine type (some derivative of Runner_, handled automagically)
    /// \param evt The event that put us in this state
    /// \param fsm The command runner
    template <class Event, class FSM>
    void on_exit(Event const& evt, FSM& fsm)
    {
      if (fsm.sm)
      {
        fsm.sm->_cancelTimeout_nolock();
      }
    }
  };

  /// @brief Delay after completing a command and before starting the next one
  struct PostDelay : public msm::front::state<>
  {
    /// @brief Function called on entering the "wait to send data" state
    ///
    /// Setups up the timeout to fire when appropriate
    ///
    /// \tparam Event Event Type (handled automagically)
    /// \tparam FSM Finite state machine type (some derivative of Runner_, handled automagically)
    /// \param evt The event that put us in this state
    /// \param fsm The command runner
    template <class Event, class FSM>
    void on_entry(Event const&, FSM& fsm)
    {
      if (fsm.sm)
      {
        fsm.sm->_setTimeout_nolock(fsm.cmd.getDelayAfter());
      }
    }
  };

  // transition actions

  /// @brief Function called when a pre- or post- command delay expires
  void timer_done(const TimerDone&){};

  /// @brief Function called when a command timeout delay expires
  void log_timeout(const TimerDone& evt)
  {
    if (cmd.warnOnTimeout())
    {
      ROS_WARN_STREAM("I/O State Machine: WaitForData timed out! CMD=" << cmd.getCommand());
    }
    sm->_callTimeoutCallback_nolock(cmd);
  }

  /// @brief Function called when a StartCommand event is initiated
  void new_cmd(const StartCommand& _cmd)
  {
    cmd = _cmd.cmd;
  }

  // Guard Conditions-- prevents a transition from happening by
  // returning false

  /// @brief Guard Condition to prevent state transition to "received"
  ///
  /// \param d The DataRecv event object
  /// \return True for "proceed to next state", False for "don't"
  bool data_good(const DataRecv& d)
  {
    if (cmd.emit() && sm)
    {
      bool rc = sm->_dataReady_nolock(cmd, d.msg);
      if (!rc) {
        auto data = std::string(std::begin(d.msg.data), std::end(d.msg.data));
        if (cmd.errOnStateTrans()) {
	ROS_ERROR_STREAM("IoCommand: " << cmd.getCommand()
                         << " reported invalid data: " << data);
	}
      }
      return rc;
    }
    return true;
  }

  /// @typedef MSM uses the "initial_state" type as the initial state, so typedef it here
  typedef Ready initial_state;

  /// @typedef A convenience typedef to make the transition table MUCH cleaner
  typedef Runner_front r;

  /// @struct The actual transition table.  Note how its all statically typed and whatnot
  struct transition_table
      : mpl::vector<
            //    +-------------+-----------+----------------+------------------+---------------+
            //    |    Start    |  Event    |   Next         | Action           | Guard         |
            //    +-------------+-----------+----------------+------------------+---------------+
            a_row<PreDelay, TimerDone, WaitForData, &r::timer_done>,
            g_row<WaitForData, DataRecv, PostDelay, &r::data_good>,
            a_row<WaitForData, TimerDone, PostDelay, &r::log_timeout>,
            a_row<PostDelay, TimerDone, Ready, &r::timer_done>,
            a_row<Ready, StartCommand, PreDelay, &r::new_cmd>
            //    +-------------+-----------+----------------+------------------+---------------+
            >
  {
  };

  /// @brief Replaces standard no_transition function.
  template <class FSM, class Event>
  void no_transition(Event const& e, FSM&, int state)
  {
    // ROS_DEBUG_STREAM("no transition from state " << state
    //<< " on event " << typeid(e).name();
  }
};  // struct CommandRunner_

}  // namespace iosm_inner
}  // namespace ds_asio

#endif  // PROJECT_DS_IOSM_PRIVATE_H
