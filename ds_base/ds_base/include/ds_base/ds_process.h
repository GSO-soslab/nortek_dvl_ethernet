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
#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_base/ds_global.h"
#include "ds_asio/ds_asio.h"
#include "ds_core_msgs/Status.h"
#include "ds_core_msgs/CriticalProcess.h"

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/algorithm/string.hpp>

namespace ds_base
{
struct DsProcessPrivate;

/// @brief The base class for ROS nodes in Deep Submergence ROS
///
/// This class serves as the base class for all ROS nodes in the Deep Submergence
/// ROS ecosystem.
///
/// Reminder:  The `~` prefix before parameters and topics denotes "private" parameters
/// and topics.  That is, they live BELOW the fully resolved node name namespace.  So if
/// your node is named `node` in namespace `/some/namespace`, then the fully resolved node
/// namespace is `/some/namespace/node` and the parameter `~param` lives on the parameter
/// server at `/some/namespace/node/param`
///
/// # Parameters
///
/// `DsProcess` automatically checks the parameter server for the following:
///
///   - `~health_check_period`:  Period (in seconds) between checking internal status.
///   - `~descriptive_name`: A human-sensible name for the node.  Keep it short.
///   - `~uuid`: A unique, deterministic ID for the node.
///
/// # Topics
///
/// `DsProcess` automatically creates the following topics:
///
///   - `~status` (`ds_core_msgs::Status`):  Periodic status message.
///
/// # Extending
///
/// This class follows the `PIMPL` idiom, and all of the actual implementation details
/// are pushed into the protected `Private` structure assocated with the class.  This
/// provides a stable ABI and a clean public interface for users.  These benifits come
/// at some extra cost in complexity.  A detailed guide to extending `DsProcess` can
/// be found in the `EXTENDING.md` document.
class DsProcess
{
  DS_DECLARE_PRIVATE(DsProcess)

public:
  /// @brief Construct a new DsProcess
  ///
  /// When using this constructor you must call ros::init elsewhere in your code
  DsProcess();

  /// @brief Construct a new DsProcess
  ///
  /// This constructor calls ros::init(argc, argv, name) for you
  ///
  /// @param[in] argc
  /// @param[in] argv
  /// @param[in] name The name of the process type
  DsProcess(int argc, char** argv, const std::string& name);

  /// @brief Destroys a DsProcess
  virtual ~DsProcess();

  DS_DISABLE_COPY(DsProcess)

  /// @brief Access the owned DsNodeHandle
  ///
  /// @return A pointer to the protected DsNodeHandle instance. If the DsNodeHandle does not already exist,
  /// it in instantiated here
  ros::NodeHandle nodeHandle(const std::string& ns = "");

  /// @brief Run the owned asio io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run();

  /// @brief Period between process health checks.
  ///
  /// \return
  ros::Duration statusCheckPeriod() const noexcept;

  /// @brief Set the period for status health checks.
  ///
  /// A period < 0 disables health checks.
  void setStatusCheckPeriod(ros::Duration period) noexcept;

  /// @brief Period between critical process ttl broadcasts
  ///
  /// \return
  ros::Duration criticalProcessPeriod() const noexcept;

  /// @brief Set the period for critical process ttl broadcasts
  ///
  /// A period < 0 disables broadcasts
  void setCriticalProcessPeriod(ros::Duration period) noexcept;

  /// @brief Give the node a descriptive name
  ///
  /// \param name
  void setDescriptiveName(const std::string& name) noexcept;

  /// @brief Get the node's descriptive name;
  ///
  /// \return
  std::string descriptiveName() const noexcept;

  /// @brief Add an asio-based (serial, udp, etc.) connection.
  ///
  /// \param name
  /// \param callback
  /// \return
  boost::shared_ptr<ds_asio::DsConnection> addConnection(const std::string& name,
                                                         const ds_asio::ReadCallback& callback);

  /// @brief Add an asio-based I/O state machine and its associated connection (serial, UDP, etc)
  ///
  /// \param iosm_name The name of the I/O state machine (for parameter stuff)
  /// \param conn_name The name of the connection (also for parameter stuff)
  /// \param callback The callback fired when the state machine has data to send.  By default, no callback is used.
  /// \return A shared_ptr to the I/O state machine object
  boost::shared_ptr<ds_asio::IoSM> addIoSM(const std::string& iosm_name, const std::string& conn_name,
                                           const ds_asio::IoCommand::ReadCallback& callback = ds_asio::IoCommand::ReadCallback());

  /// @brief Get the UUID for the node.
  ///
  /// Generated UUID's are deterministic -- they should persist across runs
  /// with identical parameters.
  ///
  /// \return
  boost::uuids::uuid uuid() const noexcept;

  void publishStatus(const ds_core_msgs::Status& msg);

  void publishCriticalProcess(const ds_core_msgs::CriticalProcess& msg);

  /// @brief Setup node after ros has been initialized.
  ///
  /// This method is called in DsProcess' constructors, after the object has
  /// been instantiated.  It is the main entry point to add run-time configuration
  /// that requires a rosmaster to be running.
  ///
  /// Default implementation calls, in order:
  ///
  /// - setupParameters()
  /// - setupConnections()
  /// - setupSubscriptions()
  /// - setupPublishers()
  /// - setupTimers()
  /// - setupServices()
  virtual void setup();

  /// @brief Returns a pointer to the DsAsio object that DsProcess owns
  ds_asio::DsAsio* asio(void);

  /// @brief Return a ds_core_msgs::Status message
  ///
  /// Default implementation doesn't do any actual health checks, it only
  /// fills out some book-keeping:
  ///
  /// - Fills in the uuid.
  /// - Sets the descriptive name
  /// - Marks the status as 'good'
  ///
  /// \param event
  virtual ds_core_msgs::Status statusMessage();

  ds_core_msgs::CriticalProcess criticalProcessMessage();

protected:
  /// @brief Get parameters from server
  ///
  /// The default implementation looks for the following PRIVATE parameters:
  ///  - sets the current time.
  ///  - health_check_period [double]
  ///  - descriptive_name [string]
  ///  - uuid [string]
  virtual void setupParameters();

  /// @brief Create asio connections
  virtual void setupConnections()
  {
  }

  /// @brief Create ros topic subscriptions.
  virtual void setupSubscriptions()
  {
  }

  /// @brief Create ros topic publishers.
  ///
  /// The default implementation creates the following publishers:
  ///  - status  [ds_core_msgs::Status]
  virtual void setupPublishers();

  /// @brief Create ros services
  ///
  virtual void setupServices()
  {
  }

  /// @brief Create ros timers on startup.
  ///
  /// The default implementation does nothing.
  ///
  /// \param base

  virtual void setupTimers()
  {
  }

  /// @brief Check the process status.
  ///
  /// This method is triggered by the status check timer.  The default
  /// implementation calls DsProcess::statusMessage() and publishes the result.
  ///
  /// This is where you can add hooks to check process-specific details
  /// and emit a ds_core_msgs::Status message.
  ///
  /// \param event
  virtual void checkProcessStatus(const ros::TimerEvent& event);

  /// @brief Publishes a ttl message if this is a critical process
  ///
  /// This method is triggered by the critical process timer.
  virtual void onCriticalProcessTimer(const ros::TimerEvent& event);

  /// @brief Override the uuid read from the parameter server
  ///
  /// \return
  void setUuid(const boost::uuids::uuid& uuid) noexcept;

  void setTtl(int ttl);

  int getTtl(void);

private:
  std::unique_ptr<DsProcessPrivate> d_ptr_;
};
}
#endif
