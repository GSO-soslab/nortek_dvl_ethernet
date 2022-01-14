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

//#include <ds_asio/ds_iosm.h>
#include "ds_iosm_private.h"

namespace ds_asio
{
////////////////////////////////////////////////////////////////////////////////
// IoCommand
////////////////////////////////////////////////////////////////////////////////
ds_asio::IoCommand::IoCommand(const std::string& cmdstr, double timeout_sec, bool _force_next,
                              const ds_asio::IoCommand::ReadCallback& cb)
  : cmd(cmdstr), callback(cb)
{
  emitOnMatch = true;
  timeoutWarn = false;
  forceNext = _force_next;
  stateTransErr = true;
  timeout = ros::Duration(timeout_sec);
}

ds_asio::IoCommand::IoCommand(double timeout_sec)
{
  cmd = "";
  emitOnMatch = false;
  timeoutWarn = false;
  forceNext = false;
  stateTransErr = false;
  timeout = ros::Duration(timeout_sec);
}

ds_asio::IoCommand::IoCommand(const ds_core_msgs::IoCommand& _cmd)
{
  emitOnMatch = _cmd.emitOnMatch;
  timeoutWarn = _cmd.timeoutWarn;
  forceNext = _cmd.forceNext;
  stateTransErr = _cmd.stateTransErr;

  delayBefore = ros::Duration(_cmd.delayBefore_ms / 1000.0);
  delayAfter = ros::Duration(_cmd.delayAfter_ms / 1000.0);
  timeout = ros::Duration(_cmd.timeout_ms / 1000.0);

  id = _cmd.id;
  cmd = _cmd.command;
}

// getter / setters
void ds_asio::IoCommand::setCommand(const std::string& _cmd)
{
  cmd = _cmd;
}
const std::string& ds_asio::IoCommand::getCommand() const
{
  return cmd;
}

void ds_asio::IoCommand::setDelayBefore(const ros::Duration& _d)
{
  delayBefore = _d;
}
const ros::Duration& ds_asio::IoCommand::getDelayBefore() const
{
  return delayBefore;
}

void ds_asio::IoCommand::setDelayAfter(const ros::Duration& _d)
{
  delayAfter = _d;
}
const ros::Duration& ds_asio::IoCommand::getDelayAfter() const
{
  return delayAfter;
}

void ds_asio::IoCommand::setTimeout(const ros::Duration& _t)
{
  timeout = _t;
}
const ros::Duration& ds_asio::IoCommand::getTimeout() const
{
  return timeout;
}

bool ds_asio::IoCommand::emit() const
{
  return emitOnMatch;
}
void ds_asio::IoCommand::setEmit(bool _e)
{
  emitOnMatch = _e;
}

bool ds_asio::IoCommand::warnOnTimeout() const
{
  return timeoutWarn;
}
void ds_asio::IoCommand::setWarnOnTimeout(bool _w)
{
  timeoutWarn = _w;
}

bool ds_asio::IoCommand::errOnStateTrans() const
{
  return stateTransErr;
}
void ds_asio::IoCommand::setErrOnStateTrans(bool _err)
{
  stateTransErr = _err;
}

bool ds_asio::IoCommand::getForceNext() const
{
  return forceNext;
}
void ds_asio::IoCommand::setForceNext(bool _fn)
{
  forceNext = _fn;
}

uint64_t ds_asio::IoCommand::getId() const
{
  return id;
}

void ds_asio::IoCommand::setId(uint64_t _i)
{
  id = _i;
}

bool ds_asio::IoCommand::hasCallback() const
{
  return !callback.empty();
}

const ds_asio::IoCommand::ReadCallback& ds_asio::IoCommand::getCallback() const
{
  return callback;
}

void ds_asio::IoCommand::setCallback(const ds_asio::IoCommand::ReadCallback& _cb)
{
  callback = _cb;
}
const ds_asio::IoCommand::TimeoutCallback& ds_asio::IoCommand::getTimeoutCallback() const
{
  return timeoutCallback;
}

void ds_asio::IoCommand::setTimeoutCallback(const ds_asio::IoCommand::TimeoutCallback& _cb) {
  timeoutCallback = _cb;
}

////////////////////////////////////////////////////////////////////////////////
// _IoSM_impl (see ds_iosm_private.h)
////////////////////////////////////////////////////////////////////////////////
IoSM::IoSM(boost::asio::io_service& io_service, std::string name, const ds_asio::IoCommand::ReadCallback& callback)
{
  // _IoSM_impl forces us to use create, because it really has to be a shared_ptr
  impl = ds_asio::iosm_inner::_IoSM_impl::create(io_service, name, callback);
  impl->start();
}

IoSM::IoSM(boost::asio::io_service& io_service, std::string name)
{
  impl = ds_asio::iosm_inner::_IoSM_impl::create(io_service, name, ds_asio::IoCommand::ReadCallback());
  impl->start();
}
IoSM::~IoSM()
{
  // This will stop stuff from running, but if there are any
  // pending events or whatever that still have a pointer to the
  // implementation, those won't be deallocated until it is safe to do so.
  impl->shutdown();
  impl.reset();
}

void IoSM::setConnection(const boost::shared_ptr<DsConnection>& conn)
{
  impl->setConnection(conn);
}

uint64_t IoSM::addRegularCommand(const IoCommand& cmd)
{
  return impl->addRegularCommand(cmd);
}

void IoSM::deleteRegularCommand(const uint64_t id)
{
  impl->deleteRegularCommand(id);
}

bool IoSM::overwriteRegularCommand(const uint64_t id, const IoCommand& cmd)
{
  return impl->overwriteRegularCommand(id, cmd);
}

void IoSM::addPreemptCommand(const IoCommand& cmd)
{
  impl->addPreemptCommand(cmd);
}

const boost::shared_ptr<DsConnection>& IoSM::getConnection() const
{
  return impl->getConnection();
}

void IoSM::setCallback(const ds_asio::IoCommand::ReadCallback& cb)
{
  impl->setCallback(cb);
}

const ds_asio::IoCommand::ReadCallback& IoSM::getCallback() const
{
  return impl->getCallback();
}

std::list<ds_asio::IoCommand> IoSM::getRegularCommands() const
{
  return impl->getRegularCommands();
}

size_t IoSM::getPreemptQueueSize() const {
  return impl->getPreemptQueueSize();
}

////////////////////////////////////////////////////////////////////////////////
// _IoSM_impl (see ds_iosm_private.h)
////////////////////////////////////////////////////////////////////////////////
using namespace iosm_inner;
//--------------------------------------------
// Overhead
_IoSM_impl::_IoSM_impl(boost::asio::io_service& io_service, std::string n,
                       const boost::function<bool(ds_core_msgs::RawData)>& callback)
  : io_service_(io_service), timeoutTimer_(io_service), callback_(callback), name_(n)
{
  nextCmdId = 1;
  currCommand = regularCommands.end();
  commandRunning = false;
  isPreemptCommand = false;
}

_IoSM_impl::~_IoSM_impl()
{
  if (runner)
  {
    runner->invalidateSm();
  }
}

std::shared_ptr<_IoSM_impl> _IoSM_impl::create(boost::asio::io_service& io_service, std::string name,
                                               const boost::function<bool(ds_core_msgs::RawData)>& callback)
{
  return std::shared_ptr<_IoSM_impl>(new _IoSM_impl(io_service, name, callback));
}

void _IoSM_impl::start()
{
  runner.reset(new Runner(shared_from_this()));
  runner->start();
}

void _IoSM_impl::shutdown()
{
  // cancel any possibly-pending timer events
  timeoutTimer_.cancel();

  // disconnect from the connection
  connection_->setCallback(ds_asio::IoCommand::ReadCallback());

  // Clear out the I/O state machine
  runner->invalidateSm();
}

void _IoSM_impl::setConnection(const boost::shared_ptr<ds_asio::DsConnection>& conn)
{
  connection_ = conn;
  connection_->setCallback(boost::bind(&_IoSM_impl::_dataCallback, this, _1));
}

const boost::shared_ptr<DsConnection>& _IoSM_impl::getConnection() const
{
  return connection_;
}

void _IoSM_impl::setCallback(const ds_asio::IoCommand::ReadCallback& cb)
{
  callback_ = cb;
}

const ds_asio::IoCommand::ReadCallback& _IoSM_impl::getCallback() const
{
  return callback_;
}

//--------------------------------------------
// Public interface
uint64_t _IoSM_impl::addRegularCommand(const IoCommand& cmd)
{
  uint64_t id;
  {
    // auto unlocks in destructor
    std::lock_guard<std::mutex> queueLock(_queueMutex);

    id = nextCmdId++;
    regularCommands.push_back(cmd);
    // do this AFTER adding-- can't modify cmd directly
    regularCommands.back().setId(id);
  } // end of scope for queueLock; queueMutex unlocked
  // (Possibly) run the next command
  _runNextCommand();

  return id;
}

void _IoSM_impl::deleteRegularCommand(const uint64_t id)
{
  std::lock_guard<std::mutex> queueLock(_queueMutex);  // auto unlocks

  // remove is tricky. We have to ensure that the "next" iterator isn't left dangling
  std::list<ds_asio::IoCommand>::iterator iter;
  for (iter = regularCommands.begin(); iter != regularCommands.end(); iter++)
  {
    if (iter->getId() == id)
    {
      // ok, we found it.  Now the all-important check!
      if (iter == currCommand)
      {
        currCommand--;
        if (currCommand == regularCommands.end() && regularCommands.begin() != iter)
        {
          // the first check wraps the commands around as expected.
          // the second check prevents a dangling iterator when
          // the command queue has length 1
          currCommand = regularCommands.end();
        }
      }

      regularCommands.erase(iter);

      return;
    }  // if this is the right element
  }    // for all elements
}

bool _IoSM_impl::overwriteRegularCommand(const uint64_t id, const IoCommand& cmd)
{
  std::lock_guard<std::mutex> queueLock(_queueMutex);  // auto unlocks

  // The runner class keeps its own copy of anything currently being run.
  // As long as we can lock the mutex (above!) there is no issue with
  // overwriting a current command.
  std::list<ds_asio::IoCommand>::iterator iter;
  for (iter = regularCommands.begin(); iter != regularCommands.end(); ++iter)
  {
    if (iter->getId() == id)
    {
      *iter = cmd;
      (*iter).setId(id);
      return true;
    }
  }
  return false;
}

void _IoSM_impl::addPreemptCommand(const IoCommand& cmd)
{
  {
    std::lock_guard<std::mutex> queueLock(_queueMutex);  // auto unlocks
    preemptCommands.push_back(cmd);
  }  // queueLock out of scope; _queueMutex unlocked
  // (Possibly) run the next command
  _runNextCommand();
}

std::list<ds_asio::IoCommand> _IoSM_impl::getRegularCommands()
{
  std::lock_guard<std::mutex> queueLock(_queueMutex);  // auto unlocks
  std::list<ds_asio::IoCommand> ret(regularCommands);
  return ret;
}

size_t _IoSM_impl::getPreemptQueueSize() {
  std::lock_guard<std::mutex> queueLock(_queueMutex);  // auto unlocks
  return preemptCommands.size();
}

//--------------------------------------------
// Command-Runner interface
bool _IoSM_impl::_dataReady_nolock(const ds_asio::IoCommand& cmd, const ds_core_msgs::RawData& raw)
{
  // TODO: This should post an event, rather than call the callback directly
  // call the statemachine-wide callback (if it has one)
  bool rc = true;

  if (!callback_.empty())
  {
    if (!callback_(raw)) {
      rc = false;
    }
  }

  // Call the command-specific callback (if it has one)
  if (cmd.hasCallback())
  {
    if (!cmd.getCallback()(raw)) {
      rc = false;
    }
  }

  std::string str(reinterpret_cast<const char*>(raw.data.data()), raw.data.size());
  return rc;
}

void _IoSM_impl::_callTimeoutCallback_nolock(const ds_asio::IoCommand& cmd) {
  // TODO: Like the _dataReady_nolock function, this should really post an event
  // rather than call the callback directly
  if (cmd.getTimeoutCallback()) {
    cmd.getTimeoutCallback()();
  }
}

void _IoSM_impl::_sendData_nolock(const std::string& data)
{
  if (!data.empty())
  {
    connection_->send(boost::shared_ptr<std::string>(new std::string(data)));
  }
}

void _IoSM_impl::_setTimeout_nolock(const ros::Duration& timeout)
{
  boost::posix_time::time_duration duration = timeout.toBoost();
  if (timeout.toNSec() <= 0)
  {
    // timeout immediately
    std::lock_guard<std::recursive_mutex> eventLock(_processEventMutex);
    runner->process_event(TimerDone());
    return;
  } // eventLock out of scope; processEventMutex lock count decrements

  // schedule a call to timeoutCallback in the correct timeframe
  timeoutTimer_.expires_from_now(timeout.toBoost());
  timeoutTimer_.async_wait(
      boost::bind(&ds_asio::iosm_inner::_IoSM_impl::_timeoutCallback, this, boost::asio::placeholders::error));
}

void _IoSM_impl::_cancelTimeout_nolock()
{
  timeoutTimer_.cancel();
}

void _IoSM_impl::_timeoutCallback(const boost::system::error_code& error)
{
  if (error.value() == boost::system::errc::operation_canceled)
  {
    ROS_DEBUG_STREAM("I/O SM Timer operation cancelled: " << error.message());
    return;
  }

  if (error)
  {
    ROS_DEBUG_STREAM("I/O SM ERROR waiting for timer: " << error.message());
  }

  std::lock_guard<std::recursive_mutex> eventLock(_processEventMutex);
  runner->process_event(TimerDone());
}

void _IoSM_impl::_dataCallback(const ds_core_msgs::RawData& raw)
{
  std::lock_guard<std::recursive_mutex> eventLock(_processEventMutex);
  runner->process_event(DataRecv(raw));
}

void _IoSM_impl::_commandDone()
{
  {
    std::lock_guard<std::mutex> lg(_queueMutex);

    commandRunning = false;
    if (isPreemptCommand) {
      preemptCommands.pop_front();
      // The batman version posted an event here.
    }
  } // end of scope for lock_guard; _queueMutex unlocked
  _runNextCommand();
}

void _IoSM_impl::_runNextCommand()
{
  // Next command to run; this needs to be set with _queueMutex locked,
  // but used with _processEventMutex locked.
  std::unique_ptr<ds_asio::IoCommand> nextCommand;
  {
    std::lock_guard<std::mutex> queueLock(_queueMutex);
    // don't start another command if one is already running
    if (commandRunning || !runner) {
      return;
    }

    // preempt commands have priority, unless regular commands have force next set.
    if (preemptCommands.size() > 0 &&
        (isPreemptCommand || !runner->cmd.getForceNext())) {
      isPreemptCommand = true;
      commandRunning = true;
      nextCommand = std::unique_ptr<ds_asio::IoCommand>(new ds_asio::IoCommand(preemptCommands.front()));
    } else if (regularCommands.size() > 0) {
      isPreemptCommand = false;
      if (currCommand != regularCommands.end()) {
        currCommand++;
      }
      if (currCommand == regularCommands.end()) {
        currCommand = regularCommands.begin();
      }
      if (currCommand != regularCommands.end()) {
        commandRunning = true;
        nextCommand = std::unique_ptr<ds_asio::IoCommand>(new ds_asio::IoCommand(*currCommand));
      }
    }
  } // queueLock out of scope; _queueMutex unlocked

  if(nextCommand) {
    std::lock_guard<std::recursive_mutex> eventLock(_processEventMutex);
    runner->process_event(StartCommand(*nextCommand));
  }  // eventLock out of scope; _processEventMutex decrements lock count
}

};  // namespace ds_asio
