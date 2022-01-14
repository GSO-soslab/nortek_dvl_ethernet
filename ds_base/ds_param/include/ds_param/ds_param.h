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

#ifndef PROJECT_DS_PARAM_H
#define PROJECT_DS_PARAM_H

#include <memory>

#include <ds_core_msgs/ParamUpdate.h>
#include <ros/ros.h>
#include <boost/optional.hpp>

namespace ds_param
{
// forward declaration: see ds_param_connection.h
class ParamConnection;
class ParamConnectionPrivate;
class UpdatingParamPrivate;

enum UpdatingParamTypes
{
  PARAM_UNKNOWN = 0,
  PARAM_BOOL,
  PARAM_INT,
  PARAM_FLOAT,
  PARAM_DOUBLE,
  PARAM_STRING,
  PARAM_ENUM,
  PARAM_CUSTOM = 1000  // probably never used
};

/// \brief The abstract base class for all our parameter types.
///
/// Can be useful for type erasure, etc
class UpdatingParam
{
public:
  virtual ~UpdatingParam() = 0;

  /// \brief Get a YAML block description of this object.  Exact contents vary with type
  virtual std::string YamlDescription() const;

  /// \brief Get a string describing this type (also available in the YAML)
  virtual std::string Type() const = 0;

  /// \brief Get a description of the type for easy switch statements
  virtual UpdatingParamTypes TypeNum() const;

  /// \brief Get the fully-resolved ROS name for this type
  const std::string& Name() const;

  /// \brief Returns true if the variable has not been committed yet because the connection is locked
  bool IsDirty() const;

  /// \brief Load this parameter from the server.  Called automatically when instantiated with a connection
  virtual void loadFromServer() = 0;

  /// \brief Set this parameter on the server.  Called automatically when setting a variable.  You don't have to do
  /// this.
  virtual void setOnServer() = 0;

  /// \brief Fill an update message
  virtual void fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const = 0;

  /// \brief Get the advertise flag
  ///
  /// As a rule, individual programs should only advertise parameters they listen to, not parameters they
  /// set.  That way a configuring utilities don't populate the configuration options list
  ///
  /// \return Wether this process advertises this variable or not
  bool Advertise() const;
  void setAdvertise(bool v);

  bool operator==(const UpdatingParam& p) const;
  bool operator!=(const UpdatingParam& p) const;

protected:
  // only constructor is protected; as this class is pure-virtual,
  // it can only be instantiated by a child class anyway
  UpdatingParam(const std::shared_ptr<UpdatingParamPrivate>& impl);

  std::shared_ptr<UpdatingParamPrivate> d_ptr_;

private:
  auto d_func() noexcept -> UpdatingParamPrivate*;
  auto d_func() const noexcept -> UpdatingParamPrivate const*;
};

// This class will ONLY work for a limited set of types

// forward declaration
template <typename T>
class UpdatingParamTPrivate;

template <typename T>
class UpdatingParamT : public UpdatingParam
{
protected:
  UpdatingParamT(const std::shared_ptr<UpdatingParamTPrivate<T> >& impl);

public:
  UpdatingParamT(const std::shared_ptr<ParamConnectionPrivate>& conn, const std::string& name, bool advertise);

  typedef T ValueType;
  typedef std::shared_ptr<UpdatingParamT<T> > Ptr;
  typedef std::shared_ptr<const UpdatingParamT<T> > ConstPtr;

  /// \brief Get the current value of this parameter
  const T& Get() const;

  /// \brief Set the value of this parameter and tell the world
  void Set(const T& _v);

  /// \brief Get the previous value this parameter had.
  ///
  /// \return The previous value this parameter had, or no value if no previous values have been observed
  boost::optional<T> GetPrevious() const;

  virtual std::string YamlDescription() const override;
  virtual std::string Type() const;
  virtual UpdatingParamTypes TypeNum() const;

  virtual void loadFromServer();
  virtual void setOnServer();
  virtual void fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const;

protected:
  // updates the value but does NOT trigger updates; ONLY to be used internally (hence protected)
  void updateValue(const T& _v);
  friend ParamConnectionPrivate;  // needed to use updateValue

private:
  auto d_func() noexcept -> UpdatingParamTPrivate<T>*;
  auto d_func() const noexcept -> UpdatingParamTPrivate<T> const*;
};

// forward declaration
class UpdatingParamEnumPrivate;

/// \brief A class to add some discovery options for enums on top of the standard
/// int implementation
///
/// Note that current enum values are NOT shared between different instances of the same value!
/// That's a pretty major TODO
class UpdatingParamEnum : public UpdatingParamT<int>
{
public:
  UpdatingParamEnum(const std::shared_ptr<ParamConnectionPrivate>& conn, const std::string& name, bool advertise);

  typedef std::shared_ptr<UpdatingParamEnum> Ptr;
  typedef std::shared_ptr<const UpdatingParamEnum> ConstPtr;

  // getters, setters, type, etc are all inherited
  virtual std::string YamlDescription() const override;
  virtual std::string Type() const;
  virtual UpdatingParamTypes TypeNum() const;

  void addNamedValue(const std::pair<std::string, int>& value);
  void addNamedValue(const std::string& name, int value);
  const std::vector<std::pair<std::string, int> >& getNamedValues() const;
  std::vector<std::pair<std::string, int> >& getNamedValues();

  std::string getValueByName() const;
  void setValueByName(const std::string& name);
  bool hasNamedValue(const std::string& name) const;

  typedef boost::function<void(const UpdatingParamEnum& param)> Callback;
  void setCallback(const Callback& _cb);

protected:
  friend ParamConnectionPrivate;  // need to use updateValue

private:
  auto d_func() noexcept -> UpdatingParamEnumPrivate*;
  auto d_func() const noexcept -> UpdatingParamEnumPrivate const*;
};

typedef UpdatingParamT<bool> BoolParam;
typedef UpdatingParamT<int> IntParam;
typedef UpdatingParamT<float> FloatParam;
typedef UpdatingParamT<double> DoubleParam;
typedef UpdatingParamT<std::string> StringParam;
typedef UpdatingParamEnum EnumParam;
}

#endif  // PROJECT_DS_PARAM_H
