# Extending the `ds_base` package

`ds_base::DsProcess` attempts to serve as an "interface" for ROS nodes in the `ds` ecosystem.  It
defines a small number of methods that all nodes must provide (like a traditional interface) and
handles setting up the parameters, topics, etc. that all nodes are expected to provide.

This document explains how to subclass `ds_base::DsProcess` to create a new node that place nicely
with the rest of the `ds` ros ecosystem.


## Quick Rules for New ds_base::DsProcess-based Nodes

1. The new sensor must subclass `ds_base::DsProcess`
2. *Ideally* the new sensor should hide all implementation details within a subclass named
   with a `Private` suffix.  (e.g. for `DerivedClass`, another class named `DerivedClassPrivate`)
 
`ds_base::DsProcess` uses the PIMPL idom to hide implementation-specific details from the user.
"Implementation-specific" in this context refers to anything you might consider holding `protected`
or `private` that is *non-virtual* in your class.  It isn't strictly necessary to follow this pattern in your own
classes, but doing so has some benefits that include:

- Setting up ros-related things (parameters, topics, subscriptions) is done for you IF you
  override the respective methods in `DsProcess`
- Maintaining a stable ABI for your library is much, much easier.  This is less of an issue if
  you're just creating executables.
  
We use a set of macros based on those available in Qt.  Qt also provides a very nice wiki page for using
their macros with PIMPL:  [D-Pointer](http://wiki.qt.io/D-Pointer)
  
## Step 0:  Read the Source Files!

- `include/ds_process/ds_process.h`
- `src/ds_process/ds_process_private.h`

Seriously, read the header files.  There will be a *lot* more detail in those two files than this one.
And while this document will strive to keep up to date with changes to `DsProcess` it may fall out
of sync at times.  Go read the two source files, then continue.

## Step 0:  Define What You're Building

Yes, this is another step 0.  This is on purpose.  Don't start coding BEFORE you have a decent idea
of what you want.  Let's create an example process that:

- Reads data over a serial port.
- Publishes data on a topic: 'output'
- Looks for a few parameters on the parameter server.
- Periodically publishes on a second topic 'time' based on a timer.

We'll call this project "example".

## Step 1: Start With the Bare Minimum

Create three new files:

- `include/example/example.h`
- `src/example/example_private.h`
- `src/example/example.cpp`

Here's the public header file.

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

Those `DS_*` macros are defined in `include/ds_base/ds_global.h` and are heavily influenced (copied) from
the similarly named `Q_` macros in Qt's `qglobal.h` (see [D-Pointer](http://wiki.qt.io/D-Pointer)) 
They help us set up and use the PIMPL structure but require some buy-in from you:

- The private class must be the same name as the public class with `Private` added to the end
- The private unique pointer should be named `d_ptr_`
- The public destructor must be defined in the source code (not the header).

Here's the private implementation header.  NOTE: that this header file lives in `src`.  This is on purpose,
the whole point is to not export the details of the implementation in a public header.  You could make it public
by placing it in `include` if you want other people to be able to derive subclasses in their own libraries.

```C++
// file: src/example/example_private.h

// Not much to do here yet..
struct ExamplePrivate:
{
}
```

And finally here's the source file:

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

Example::~Example() = default;
```

Not too interesting yet.  We'll get there.  Right now we've got a new class that builds from `DsProcess`
but doesn't actually do anything new.

## Rule 2:  Handling ROS interaction during setup.

If you look at `include/ds_process/ds_process.h` you'll see a `DsProcess::setup` method, 
and lots of setup-helpers, like `DsProcess::setupConnections`.  As the header describes, 
`DsProcess::setup` is just a wrapper around all of the smaller setup functions.  This gives us
a good deal of modularity.  If we only care about adding a new parameter lookup we just need to override the 
paramter setup function.  If we need to add setup directives that don't fit nicely into one of the helper
functions then we can always override `setup` itself and add what we need.

**IMPORTANT:** the `DsProcess::setup` can be called manually, or left until `DsProcess::run` (take a look).

So let's go back to our "design spec":

- Reads data over a serial port.
- Publishes data on a topic: 'output'
- Looks for a few parameters on the parameter server.
- Periodically publishes on a second topic 'time' based on a timer.

We need to add:

- two publishers
- an asio connection (with callback function)
- a timer (with callback function)

### Adding the Publishers

Let's add the publishers first:

First, we put the publisher objects that will be created in the private structure.
```C++
// file: src/example/example_private.h

// Not much to do here yet..
struct ExamplePrivate:
{
  // Our actual publisher objects.
  ros::Publisher out_pub_;
  ros::Publisher time_pub_;
}
```

Then override the `setupPublishers` method in our public class.
```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
 protected:
  // Overriding the publisher setup.
  void setupPublishers() override;
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

And finally do the work of setting up our publishers
```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

Example::~Example() = default;

void Example::setupPublishers()
{
  // Do any setup required by the base class first.
  DsProcess::setupPublishers();
  
  // Another helper macro gets us a pointer to the ExamplePrivate struct named `d`
  DS_D(Example);
  
  // Setup our publishers
  auto nh = nodeHandle();
  d->out_pub_ = nh.advertise<DataMessageType>("output", 10, false);
  d->time_pub_ = nh.advertise<TimerMessageType>("time", 10, false);
}
```

Where did that variable `d` come from?   It's produced by the `DS_D` macro.

### Adding the callbacks

Now, timers and connections require callbacks.  Typically these callbacks are *not* something
you want to expose to the public, but where to put them?

- If `Example` may be further derived and the derived class may want to override the
  default callback behavior:  *make the callback virtual protected on `Example`*
- If `Example` is not indended to be a base class, or the callback is intended to be the same
  for all classes based on `Example`: *make the callback on `ExamplePrivate`*
  
We'll do both:

- The timer callback will be the same for all `Example`-based classes
- The connection callback will be virtual and able to be overriden.

Here's the new public header

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
 protected:
  // Overriding the publisher setup.
  void setupPublishers() override;
  
  // new connection callback - defined inline just for brevity.
  virtual void connectionCallback(ds_core_msgs::RawData& bytes) 
  {
    ROS_INFO_STREAM("Received " << bytes.data.size() << " bytes of data!");
  }
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

And the private header.
```C++
// file: src/example/example_private.h

struct ExamplePrivate
{

  // Called form the timer - same for all `Example` classes
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
  ros::Timer timer_;        //!< Need to save our timer object now too.
  ros::Publisher out_pub_;  //!< Our publisher for outgoing data
  ros::Publisher time_pub_; //!< Our publisher for the timer callback.
}

```

### Adding the connection and timer

Override the `setupConnections` and `setupTimers` methods:

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
 protected:
  // Overriding the publisher setup.
  void setupPublishers() override;
  void setupConnections() override;
  void setupTimers() override;
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

And now the actual setup logic.

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

Example::~Example() = default;

void Example::setupPublishers()
{
  // Do any setup required by the base class first.
  DsProcess::setupPublishers();
  
  // Another helper macro gets us a pointer to the ExamplePrivate struct named `d`
  DS_D(Example);
  
  // Setup our publishers
  auto nh = nodeHandle();
  d->out_pub_ = nh.advertise<DataMessageType>("output", 10, false);
  d->time_pub_ = nh.advertise<TimerMessageType>("time", 10, false);
}

void Example::setupConnections()
{

  // Do any setup required by the base class first.
  DsProcess::setupConnections();
  
  // Return a reference to the internal connection map structure of DsProcess
  auto connections_ = connections();
  
  // Boost-bind is used in the 'standard' ros way of providing a callback using a class method.
  connections_["connection_name"] = 
    addConnection("connection_name", boost::bind(&Example::connectionCallback, this, _1))
}

void Example::setupTimers()
{

  // Do any setup required by the base class first.
  DsProcess::setupTimers();
  
  DS_D(Example);
  
  // Just like the connection callback, we now use 'd' instead of 'this' in the boost::bind
  auto nh = nodeHandle();
  d->timer_ = nh.createTimer(ros::Duration(1), boost::bind(&ExamplePrivate::timerCallback, d, _1));
}
```

### Adding the parameter lookups

This is all good.  But let's say we want to parameterized some settings:

- The connection name should be read from the parameter server
- The timer period duration should be read form the parameter server

Let's enable that ability.  First we need a place to store these values we
retrieve from the parameter server:

```C++
// file: src/example/example_private.h

struct ExamplePrivate
{

  // Called form the timer - same for all `Example` classes
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
  std::string connection_name_ //!< Our connection name
  double timer_period_         //!< The timer period.
  ros::Timer timer_;           //!< Need to save our timer object now too.
  ros::Publisher out_pub_;     //!< Our publisher for outgoing data
  ros::Publisher time_pub_;    //!< Our publisher for the timer callback.
}

```

Override the `setupParameters` method:

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
 protected:
  // Overriding the publisher setup.
  void setupPublishers() override;
  void setupConnections() override;
  void setupTimers() override;
  void setupParameters() override;
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

Now, modify the class source file to retrieve those parameters and use them in further
setup directives:

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

void Example::setupParameters()
{
  DsProcess::setupParameters();
  
  DS_D(Example);
  // look for a private param named "connection_name", default to "instrument"
  d->connection_name_ = ros::param::param<std::string>("~connection_name", "instrument"); 
  
  // Similar for the timer period, default to 1 second.
  d->timer_period_ = ros::param::param<double>("~timer_period", 1);
}
  
void Example::setupPublishers()
{
  // Do any setup required by the base class first.
  DsProcess::setupPublishers();
  
  // Another helper macro gets us a pointer to the ExamplePrivate struct named `d`
  DS_D(Example);
  
  // Setup our publishers
  auto nh = nodeHandle();
  d->out_pub_ = nh.advertise<DataMessageType>("output", 10, false);
  d->time_pub_ = nh.advertise<TimerMessageType>("time", 10, false);
}

void Example::setupConnections()
{

  // Do any setup required by the base class first.
  DsProcess::setupConnections();
  
  // Return a reference to the internal connection map structure of DsProcess
  auto connections_ = connections();
  
  DS_D(Example);
  // Boost-bind is used in the 'standard' ros way of providing a callback using a class method.
  connections_[d->connection_name_] = 
    addConnection(d->connection_name_, boost::bind(&Example::connectionCallback, this, _1))
}

void Example::setupTimers()
{

  // Do any setup required by the base class first.
  DsProcess::setupTimers();
  
  DS_D(Example);
  
  // Just like the connection callback, we now use 'd' instead of 'this' in the boost::bind
  auto nh = nodeHandle();
  d->timer_ = nh.createTimer(d->timer_period_, boost::bind(&ExamplePrivate::timerCallback, d, _1));
}
```

### Adding directives that don't quite fit...

As mentioned, you're not limited to the `setup`* methods.  You can override `setup()` itself!  Let's do
that to start the timer after everything else is done:

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

// Forward-declare our private implementation
struct ExamplePrivate;

class Example : public ds_base::DsProcess {

 // Use a helper macro to set up accessor functions for our private implementation
 DS_DECLARE_PRIVATE(Example)
 
 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);
  // Need to specify the destructor in the source file so that EamplePrivate will be
  // fully-defined before-hand (needed because we're using unique_ptr's)
  ~Example() override;
  
  // Helper to disable copies.
  DS_DISABLE_COPY(Example)
  
  // Setup is a *public* function!
  void setup() override;
  
 protected:
  // Overriding the publisher setup.
  void setupPublishers() override;
  void setupConnections() override;
  void setupTimers() override;
  void setupParameters() override;
  
 private:
 // This will hold our PIMPL  object.
 std::unique_ptr<ExamplePrivate> d_ptr_;
};

}
```

Now, modify the class source file to retrieve those parameters and use them in further
setup directives:

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate))
{
}

Example::~Example() = default;

void Example::setupParameters()
{
  DsProcess::setupParameters();
  
  DS_D(Example);
  // look for a private param named "connection_name", default to "instrument"
  d->connection_name_ = ros::param::param<std::string>("~connection_name", "instrument"); 
  
  // Similar for the timer period, default to 1 second.
  d->timer_period_ = ros::param::param<double>("~timer_period", 1);
}
  
void Example::setupPublishers()
{
  // Do any setup required by the base class first.
  DsProcess::setupPublishers();
  
  // Another helper macro gets us a pointer to the ExamplePrivate struct named `d`
  DS_D(Example);
  
  // Setup our publishers
  auto nh = nodeHandle();
  d->out_pub_ = nh.advertise<DataMessageType>("output", 10, false);
  d->time_pub_ = nh.advertise<TimerMessageType>("time", 10, false);
}

void Example::setupConnections()
{

  // Do any setup required by the base class first.
  DsProcess::setupConnections();
  
  // Return a reference to the internal connection map structure of DsProcess
  auto connections_ = connections();
  
  DS_D(Example);
  // Boost-bind is used in the 'standard' ros way of providing a callback using a class method.
  connections_[d->connection_name_] = 
    addConnection(d->connection_name_, boost::bind(&Example::connectionCallback, this, _1))
}

void Example::setupTimers()
{

  // Do any setup required by the base class first.
  DsProcess::setupTimers();
  
  DS_D(Example);
  
  // Just like the connection callback, we now use 'd' instead of 'this' in the boost::bind
  auto nh = nodeHandle();
  d->timer_ = nh.createTimer(d->timer_period_, boost::bind(&ExamplePrivate::timerCallback, d, _1));
}

void Example::setup()
{
  DsProcess::setup();
  
  DS_D(Example);
  d->timer_.start();
}
```


## Interacting With the Public Class

If you need the PIMPL object to access the public class, the easiest way to do that is to
add a pointer to the public class as an argument of the private member.  For example:

In the PIMPL:

```C++

void ExamplePrivate::someMethod(Example* base, double data)
{
    // need to pass data to some public member
    base->methodThatTakesDouble(data);
}

```

Your public class:
```C++

void Example::methodThatCallsPrivate(double data)
{
    // Get a poitner to the impl
    DS_D(Example);
    
    // Call the private member, which in turn will call the next
    // method defined below.
    d->someMethod(this, data);
}

void Example::methodThatTakesDouble(double data)
{
    // does something interesting here
}

```

So calling `Example::methodThatCallsPrivate(data)` executes the following in order:

- `Example::methodThatCallsPrivate(data)`
- `Example::Impl::someMethod(data)`
- `Example::methodThatTakesDouble(double data)`


Another method is to place a bare-pointer to the public class in `ExamplePrivate`.  This works
if you're class is *non-copyable* (you never have to worry about `Example` and `ExamplePrivate`)
getting out of sync.  Then you could do stuff like:


```C++
// file: src/example/example_private.h

#include "include/example/example.h"

struct ExamplePrivate:
{

  DS_DECLARE_PUBLIC(Example)
  
  ExamplePrivate(Example* public)
    : q_ptr_(public)
  {
  }
  
  // Don't need to clean up q_ptr_, it outlasts us.
  ~ExamplePrivate() = default;
  
  Example* q_ptr_;
}
```

Then modify the constructors in `example.cpp`:

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

// Public default constructor:  use our own protected anolog
Example::Example() 
  : DsProcess()
  , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate(this)))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ExamplePrivate>(new ExamplePrivate(this)))
{
}

Example::~Example() = default;
```

Then you can access the public class pointer using the `DS_Q` macro (which will create a `q` variable):

```C++
struct ExamplePrivate:
{

  DS_DECLARE_PUBLIC(Example)
  
  ExamplePrivate(Example* public)
    : q_ptr_(public)
  {
  }
  
  void callPublic()
  {
    DS_Q(Example);
    q->somePublicMethod(); 
  }
  
  // Don't need to clean up q_ptr_, it outlasts us.
  ~ExamplePrivate() = default;
  
  Example* q_ptr_;
}
```


