# ```DS_PARAM```
Dynamically reconfigurable variables, but without ```dynamic_reconfigure```.  Tailor-made to make aggregators awesome.

## The Core Concept
The Sentry ROS Upgrade design identified three major types of parameters for a node:
* Things that change quickly, like desired thruster values.  We call these "inputs" and use topics
* Things that are set at launch and remain static throughout a node's lifetime, like UDP port configurations.  We call 
these "parameters" and use the standard ROS parameter server
* Things that are initially set at launch, but can occasionally be changed-- often at the request of a human.  We call 
these "runtime parameters", and couldn't find an acceptable package.  So that's what we're talking about.

Numerous schemes were proposed.  The off-the-shelf ```dynamic_reconfigure``` package was seriously evaluated, but it was 
found to be difficult to use.  In addition, the requirement to fetch the complete node configuration, modify it, and
send back a complete state leaves ```dynamic_reconfigure``` susceptible to race conditions.  Several custom schemes for 
using services to build on the core ```dynamic_reconfigure``` concept were considered, but ultimately rejected.

## Initial Requirements
Based on all this searching, the following design requirements were developed

* Code to manage "runtime parameters" should live in a library that functions independently of all other ```ds_ros```
software.  This allows development using parameters to proceed while allowing future changes to the attribute handling
logic.  It also lets other users add ```ds_ros``` style parameters to existing ```ROS``` nodes.
* Runtime parameters load initial values from the parameter server at startup
* Runtime parameters should keep the parameter server up-to-date so that the parameter server can serve as the
definitive current state of all parameters.  This also makes it possible to preserve state when re-starting stopped
processes.
* Runtime parameters should support the basic ```ros_param``` types of bool, int, float, double, and string to start 
with
* It is not always possible to know ahead of time what the exact values of an enumeration are, especially if loading
a list of inputs at startup.  Schemes like ```dynamic_reconfigure``` that rely on this are problematic in practice
* It is very handy to make runtime parameters discoverable.  ```dynamic_reconfigure``` used the discoverability of
parameters to build a generic GUI.  There are other helpful applications for discoverability for acomms.
* That said, the team concluded generic GUIs were of dubious value in the final system.  While possibly helpful for 
testing, they are no substitute for a hand-designed GUI.  Especially at the expense of other short-term priorities.
* C++ support is a high priority, with python support on the horizon.

The initial design was based on these requirements.  Although parameter updating and basic C++ API are fleshed out,
there remain open questions on how to manage variable discovery and enums.  However, many of the behaviors can change
in ways that should not break software using the public API.

## Connecting to parameters

The ```ds_param``` package is extremely lightweight, using primarily the existing parameter server and two global 
topics.   On startup, a node creates a ```ds_param::ParamConnection``` to the system.  This class manages all callbacks
and parameter updates for ```ds_param```.  The connection is created as follows:

```objectivec
#include <ds_param/ds_param_conn.h> // all you need

// bunches of file goes here <snip>

// down in some function somewhere:

ros::NodeHandle handle; // probably created elsewhere, MUST not be private
ds_param::ParamConnection::Ptr conn = ds_param::ParamConnection::create(handle);
```

Individual variables are then accessed using the templated ```connect``` method as, for example:

```objectivec
ds_param::BoolParam   param_bool   = conn->connect<ds_param::BoolParam  >(ros::this_node::getName() + "/test_bool_param");
ds_param::IntParam    param_int    = conn->connect<ds_param::IntParam   >(ros::this_node::getName() + "/test_int_param");
ds_param::FloatParam  param_float  = conn->connect<ds_param::FloatParam >(ros::this_node::getName() + "/test_float_param");
ds_param::DoubleParam param_double = conn->connect<ds_param::DoubleParam>(ros::this_node::getName() + "/test_double_param");
ds_param::StringParam param_string = conn->connect<ds_param::StringParam>(ros::this_node::getName() + "/test_string_param");
ds_param::EnumParam   param_enum   = conn->connect<ds_param::EnumParam  >(ros::this_node::getName() + "/test_enum_param");
```

*Only* these six types may be used.  The template implementation is deliberately hidden in the library and 
explicit template instantiation is used to ensure that only the desired types will link correctly.  Attempting to use 
other types will result in template linker hell.  Unlike usual template linker hell there will be no way out.   
```EnumParam``` inherits from ```IntParam``` and can thus be used anywhere ```IntParam``` is used.  At present, it 
only exposes additional functionality to advertise name / value pairs for an enum.

The ```connect``` method takes the name of the attribute in as a string and an optional boolean parameter indicating
whether to advertise this parameter as an input to this node.  The connect method, in order, does the following:
1. Fully resolves the parameters's name using the previously-provided node handle and its namespacing
1. Checks a local cache for an existing reference to that attribute.  If found, a shared pointer is returned.
1. Loads an initial value for the attribute from the parameter server
1. If this attribute is to be advertise, updates the YAML description of the node's current parameters and publishes it

Although the connection class was designed to permit multiple ```ParamConnection``` instances in a single node, it 
is considered an anti-pattern (not recommended!).  There are several additional limitations described later in 
"Known Limitations."

## Operations on Parameters

Individual processes are largely expected to carry around concrete derived parameters rather than the abstract base 
class.  If you follow that pattern, you probably want to use the following methods:

* `Get()` Returns a const reference to the local copy of the parameter.  An `int` for `IntParam` (and `EnumParam`), 
an `std::string` for `StringParam`, and so on.
* `Set(T value)` to update parameters through the entire system, including the parameter server.  The type `T` will be 
`int` for `IntParam`, `std::string` for `StringParam`, and so on.
* `Name()` to return the fully-resolved name of this variable (VERY useful for debugging / console messages!)

In some very limited and bizarre circumstances, the following functions may also help:

* `std::string Type() const` Return a string indicating the type of the variable.  This is really intended 
for debug / console messages and YAML generation.  Using this for switch or if/else blocks is also an anti-pattern, as 
it will break hard if we ever add a new type.
* `void loadFromServer()` Force the variable to re-load from the parameter server
* `bool Advertise() const;` and `void setAdvertise(bool v);` both exist, although this should really be handled
by the initial `connect` call.
* `std::string YamlDescription() const` gives the current YAML string for this parameter.  This probably isn't *quite* 
ready for primetime just yet.

### Some notes on `Get` and `Set`

Primary access into the parameter subsystem happens through `Get` and `Set`.  A local cache of each parameter is
maintained to keep frequent calls to `Get` efficient.  That local cache is updated as soon as the `ParamConnection` is
notified of a value change.

Calls to `Set` not only update the local cache but also update the value on the parameter server and send an update 
message on the global `/updating_param/updates` topic.  After initialization, parameters are updated from that topic 
and generally do not touch the `ROS` parameter server.  Parameters managed through ```ds_param``` should only be 
updated through ```ds_param``` after nodes have been started.

### The `EnumParam` Type

It is especially helpful for an enumerated datatype to be able to describe its legal values.  Initial support for this
is provided by the `EnumParam` datatype.  This is a standard `IntParam` with an additional vector of name/value pairs
stored as `std::pair<string, int>`.  The integer value is handled through the standard `Get` and `Set` methods.  The 
following additional methods provide convenient ways to handle named-value semantics.

```objectivec
  void addNamedValue(const std::pair<std::string, int>& value);
  void addNamedValue(const std::string& name, int value);
  
  const std::vector<std::pair<std::string, int> >& getNamedValues() const;
  std::vector<std::pair<std::string, int> >& getNamedValues();

  std::string getValueByName() const;
  
  void setValueByName(const std::string& name);
  
  bool hasNamedValue(const std::string& name) const;
```

Currently, however, these names are NOT shared between `EnumParam`s running in different nodes.  This remains a work 
in progress and high on the TODO list.

The primary use-case is expected to come up in aggregators.  A navigation aggregator, for example, might read a list of
possible heading sources and construct an `EnumParam` based on those options.

## Locking the Connection

Sometimes it is advantageous to send multiple updates in a single parameter update message.  You can do this by locking
the connection prior to queue updates rather than sending them.  All queued updates are sent when the connection is 
unlocked.  Updates will continue to arrive while the connection is locked.

The recommended way to lock the connection is with a ```ParamGuard``` to manage the lock.  This ```ParamGuard``` 
implements RAII ("Resource Acquisition Is Initialization") locking semantics similar to ```std::mutex_lock```.  It will 
automatically unlock when the object goes out of scope.  You can use it as follows:

```objectivec
#include "ds_param/param_conn.h"  // standard include
#include "ds_param/param_guard.h" // Lock guard for atomic updates
// somewhere up here, declare your:
ds_param::ParamConnection::Ptr conn = ds_param::ParamConnection::create(nodeHandle);
// and connect some parameters:

{
  ds_param::ParamGuard lock(conn);
  
  // parameters that need to update in a single message
  some_var_1.Set(1);
  some_var_valid.Set(true);
  some_var_str.Set("This group of variables updates together with valid=\"true\", value=\"1\"");
  
  // lock gets deleted here because it passes out of scope
  // that automatically unlocks the connection and sends all the updates above
  // in a single message
  // or, you can:
  lock.unlock(); // manually.  If you're lame.
}
```

While the connection is locked, a call to `Set` will update the value returned by `Get` on a parameter.  The current
value will only be moved to `previous` if that current value was set prior to the connection being locked or if it has
been updated since the connection was locked.


## Notable TODOs

* Fix "advertise flag does nothing" bug
* Global on-change callback to be called when any parameter managed by a particular connection is updated
* On-change callbacks for individual parameters
* Python API / implementation
* Command-line tools for introspection (in python)
* Sharing description-related information between nodes (Enum values)
* Multi-parameter updates (atomicity!)
* Adding more description information (min/max values)
* `rqt` Plugin for introspection and manipulation
* Platform-wide parameter discovery at runtime
* Add `ds_param` node to provide enhanced system-wide introspection, variable setting, and 
periodic parameter server checks

## Known Limitations

Individual parameters only check the parameter server at startup.  Processes changing the parameter server without
going through `ds_param` can make the parameter server inconsistent with the state of synchronized `ds_param` instances.

One answer is to periodically check the parameter server for changes and fire an update if found, but it is 
unclear who should be responsible for that.

Currently, there is no way to issue or require atomic updates to multiple variables.  Issuing atomic multi-parameter
updates requires a straightfoward addition to the connection class.  Requiring atomic updates is much more difficult.  
There are naive solutions that involve checks by the connection class, but without either a centralized control node or
a way for a single node to obtain "ownership" of a parameter it is difficult to enforce consistency.  

# Where to Go From Here

Check out the example node `ds_param_example.cpp` as used by `test.launch`.

## Example YAML (SUPER-initial version):

Formatting slightly altered to fit in document.  This is generated programmatically for a reason.  It was tested and 
parses in python-yaml.

```yaml
node:
    name: /shared_param_1
    namespace: /
params: [ { name: "/shared_param_1/test_enum_param"  ,type: "int", 
                                                          enum: { "Option 1": 1,  "non-consecutive Option 7": 7, } },   
          { name: "/shared_param_1/test_int_param", type: "int" }, 
          { name: "/shared_param_1/test_str_param"  , type: "string" }, ]
```
