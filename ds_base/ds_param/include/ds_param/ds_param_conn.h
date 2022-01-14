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

#ifndef PROJECT_DS_PARAM_CONN_H
#define PROJECT_DS_PARAM_CONN_H

#include <ros/ros.h>

#include <memory>
#include <string>
#include "ds_param.h"

namespace ds_param
{
// forward declarations
class ParamConnectionPrivate;
class ParamGuard;

class ParamConnection
{
private:
  ParamConnection(ros::NodeHandle& handle);

public:
  /// \typedef A shared_ptr typedef for this class
  typedef std::shared_ptr<ParamConnection> Ptr;

  /// \typedef A handy shared_ptr-to-a-const-copy typedef for this class
  typedef std::shared_ptr<const ParamConnection> ConstPtr;

  /// \brief Create a Parameter Connection instance.  Requires a global
  /// node handle.
  static std::shared_ptr<ParamConnection> create(ros::NodeHandle& _h);

  /// \brief Destructor
  virtual ~ParamConnection();

  /// \brief Connect to an updating parameter on the parameter server
  ///
  /// Connect to the parameter server and create a live-updating parameter
  /// for the given parameter name.
  ///
  /// \tparam T The type of the parameter.  MUST be one of our standard parameter
  /// typedefs: BoolParam, IntParam, FloatParam, DoubleParam, or StringParam
  /// or your code won't link!
  /// \param param_name The name of the parameter.  This will be resolved using the current
  /// node namespace go get the full name.
  /// \param advertise Indicates whether the parameter connection should advertise
  /// this parameter as being associated with this node.  Parameters this process
  /// reads should be advertised; parameters this process sets should not be advertised
  /// \return A shared_ptr to the requested type.  If the parameter does not exist or
  /// cannot be cast to the requested type then an empty pointer (NULL) is returned
  template <typename T>
  typename T::Ptr connect(const std::string& param_name, bool advertise = true);

  /// \brief Get this connection's unique ID
  ///
  /// Every connection generates a unique name to avoid updating in response to its
  /// own updates.  You really shouldn't need or use this, but it might be handy
  /// for debugging or console messages or something
  const std::string& connName() const;

  /// \typedef A typedef for a collection of parameters.  If you use iterators and
  /// this typedef you probably won't have to rework code even if we switch to lists
  /// or certain other STL containers
  typedef std::vector<std::shared_ptr<ds_param::UpdatingParam> > ParamCollection;

  /// \typedef The callback type fired when a parameter is changed
  typedef boost::function<void(ParamCollection&)> Callback_t;

  /// \brief Set the "variable changed" callback
  ///
  /// \param _cb The callback function.  Should accept a ParamCollection as its only (visibile) parameter
  void setCallback(const Callback_t& _cb);

  /// \brief Queue updates rather than sending them out
  ///
  /// Locking an already-locked connection has no effect.  You probably don't want to use this directly--
  /// instead use ParamGuard(your_connection_ptr);
  void lock();

  /// \brief Stop queueing updates, and send the current state of all variables that
  /// have changed since we were locked
  void unlock();

  /// \brief Check if this object is locked
  bool IsLocked() const;

protected:
  std::shared_ptr<ParamConnectionPrivate> impl;
};
}
#endif  // PROJECT_DS_PARAM_CONN_H
