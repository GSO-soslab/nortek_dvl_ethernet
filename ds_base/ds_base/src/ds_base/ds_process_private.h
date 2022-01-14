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
// Created by zac on 1/9/18.
//

#ifndef DS_BASE_DS_PROCESS_PRIVATE_H
#define DS_BASE_DS_PROCESS_PRIVATE_H

#include "ds_asio/ds_asio.h"
#include "ds_base/ds_process.h"

namespace ds_base
{
/// @brief The base implementation class for ROS nodes in Deep Submergence ROS.
///
/// This class provides the core implementation details for all `DsProcess-based`
/// nodes.  At the core of this are a number of virtual methods that allow users
/// to add custom steps where needed in their node's setup:
///
///   - `DsProcess::Impl::setupParameters`:   Parameter-server based setup directives.
///   - `DsProcess::Impl::setupConnections`:  Add `DsAsio` connections here
///   - `DsProcess::Impl::setupPublishers`:   Add topic publishers here.
///   - `DsProcess::Impl::setupSubscribers`:  Add subscribers here.
///   - `DsProcess::Impl::setupTimers`:       Add timers here.
///   - `DsProcess::Impl::setupServices`:     Add services here.
///   - `DsProcess::Impl::setup`:             Wraps all of the above
///
/// This allows developers to selectively add extra configuration directives without
/// re-implementing by hand all of the parent class' stuff by hand.
///
/// **NOTE:** You **don't** have to call setup() in your own constructors.  That's the whole
/// point of this.  The base class will do it for you.
///
/// # What Goes In Here?
///
/// This class (and those derived from it) should contain as much of the non-public
/// implementation details for their associated public class.   Examples of things
/// you'll want to put here:
///
///   - Helper methods not intended to be called by users
///   - Member variables not intended to be accessed by users
///   - ... really anything you'd mark `protected:` or `private:`
///
/// There are times when you can't follow this strictly, for example if you want to
/// use some templated functions.  But try to keep as much of the details "hidden"
/// from the user.  Besides presenting a cleaner object for use, it helps keep
/// ABI promises.
struct DsProcessPrivate
{
  DsProcessPrivate();
  virtual ~DsProcessPrivate() = default;

  /// @brief Handle restarting the status check timer.
  ///
  /// This is called from DsProcess::setStatusCheckPeriod and handles restarting
  /// the actual status check timer object if needed.
  ///
  /// \param base
  /// \param period
  void updateStatusCheckTimer(DsProcess* base, ros::Duration period);

  /// @brief Handle restarting the critical process check timer.
  ///
  /// This is called from DsProcess::setCriticalCheckPeriod and handles restarting
  /// the actual critical process check timer object if needed.
  ///
  /// \param base
  /// \param period
  void updateCriticalProcessTimer(DsProcess* base, ros::Duration period);

  bool is_setup_;     //!< Has setup() been called?
  bool is_critical_;  //!< Is this a critical process that needs to publish a ttl?
  int ttl_;           //!< TTL advertised in the critical process message if the process is marked as critical

  std::unique_ptr<ds_asio::DsAsio> asio_;  //!< DsAsio instance

  ros::Duration critical_check_period_;  //!< The period for the critical process timer broadcast (<0 disables)
  ros::Timer critical_check_timer_;      //!< The critical process timer broadcast itself.
  ros::Duration status_check_period_;    //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;        //!< The status health timer itself.
  std::string descriptive_node_name_;    //!< A short, descriptive name given to the process.
  boost::uuids::uuid uuid_;              //!< UUID of node.

  ros::NodeHandlePtr my_node_handle_;       //!< We keep a copy of a node handle to ensure ROS isn't stopped / restarted
                                            /// over the lifetime of this DsProcess instance.  This is somewhat idiomatic
                                            /// in the ROS ecosystem, as various ROS objects keep a copy of their node
                                            /// handle internally (see also: the implementation of Publisher, Subscriber,
                                            /// etc).  Users should also ALWAYS use the nodeHandle creation function
                                            /// provided by DsProcess to ensure their callback queues are setup correctly.
                                            /// There isn't an issue with putting a nodeHandle in the program main,
                                            /// but its literally the only place in DsProcess where that's appropriate.
  ros::Publisher status_publisher_;            //!< The status channel publisher.
  ros::Publisher critical_process_publisher_;  //!< The ttl published is the processis critical

  std::unordered_map<std::string, ros::Time> last_published_timestamp_;  //!< Timestamp of last message sent by
                                                                         //! publisher
};
}
#endif  // DS_BASE_DS_PROCESS_PRIVATE_H
