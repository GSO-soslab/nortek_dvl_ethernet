# The `ds_asio` I/O State Machine

Many common sensors are polled.  Sensors operating on a combined bus, such as SAIL, RS-485, etc must be polled to avoid 
contention on the bus.  Some sensors require a cycle of multiple commands, occasional one-time configuration commands, 
command sequences, and so on.

Such setups benefit greatly from an off-the-shelf I/O State Machine implementation.  The I/O state machine is designed
 to handle most common use-cases and let the user focus on writing command and parsing code rather than worrying about 
 how to make delays between commands (or whatever) happen correctly.  The I/O State Machine includes a number of 
 additional features designed to make handling complex sensor idiosyncrasies easier.

The I/O state machine runs a number of commands.  Each command is represented by a `ds_asio::IoCommand`.  In general,
 each command is a single query generating a single response.

The I/O state machine and its command object are defined in the `ds_asio` namespace in `ds_iosm.h`.

## I/O State Machine Queues

The I/O State Machine currently defines two command queues.  The "regular" queue fires off commands in a regular
 sequence one after the other.  When the sequence of regular commands is complete it starts all over again at the
  beginning.  This queue is useful for regularly querying a sensor and/or bus.  Regular commands can be delete or
   overwritten as required.  The functions to manipulate regular commands are:

```$cpp
uint64_t addRegularCommand(const IoCommand& cmd);
void deleteRegularCommand(const uint64_t id);
void overwriteRegularCommand(const uint64_t id, const IoCommand& cmd);
```

Regular commands are identified inside the state machine with a unique `uint64_t`.  Users must track this ID if they
wish to delete or overwrite commands.

There is also a "preempt" queue that can send single one-time commands.  Despite the name, preempt commands will not 
interrupt a currently-executing command.  Instead, these commands are run as soon as the next slot is available and 
are then removed from the queue.  The preempt queue is commonly used to send one-time or infrequent commands to a 
device in response to user input.  For example, the Lambda power supply driver uses preempt commands to set the output
voltage when commanded by the user.  Preempt commands can only be added, and are automatically deleted after they
have run.  The function to add preempt commands is:

```$c++
void addPreemptCommand(const IoCommand& cmd);
```

Some sensors require multiple commands to happen in sequence. Some buses, for example, require an address/response 
before a query or command can be sent.  Users can specify a `forceNext` flag to prevent a Preempt command from 
interrupting such a command sequence.

## The Commands

Each command has several parameters and is also implemented as a state machine.  In order of use, these parameters are:

* `delayBefore`: A configurable delay before running the command.  Defaults to 0.
* `cmd`: The command string to be sent
* `timeout`: The maximum amount of time to wait for a reply.  If no reply is received in this amount of time, no data
is sent.
* `timeoutWarn`: A flag to send a ROS console warning message whenever a timeout occurs.
* `emitOnMatch`: A flag indicating whether to call callbacks when data arrives or simply continue to the next command.
Defaults to "true" (DO trigger an event by default).
* `callback`: A `boost::function<void(const ds_core_msgs::RawData&)>` function pointer to call when data has been 
successfully received.
* `delayAfter`: A configurable delay after the command is run.  Defaults to 0.
* `forceNext`: Force the next command to come from the same queue as this command.  Used to enforce atomic command
sequences.

There are getters and setters for each of these options.  Delays less than or equal to 0 are ignored.  Setting the 
timeout to less than or equal to zero will cause the state machine to block until input is received and is _not 
recommended_.  Two convienence constructors are available:

To make a normal command that sends and waits:
```$objectivec
/// @brief Shorthand to create a standard command/timeout pair
IoCommand(const std::string &cmdstr, double timeout_sec, bool _force_next=false,  RecvFunc callback=RecvFunc());
```

To add a static delay, for example, to slow down a polling loop:
```$objectivec
/// @brief Shorthand to create a static wait
IoCommand(double timeout_sec);
```

Default commands proved dangerous in testing, since executing a command with no transmitted data and a timeout less 
than or equal to zero will leave the I/O state machine waiting forever for data that does not come.  For this reason,no 
default constructor is provided.

## Creating a State Machine

State machines in `ds_asio` are typically associated with a `DsConnection` object.  A single method `addIoSM` is 
provided to setup the connection, the state machine, and the callbacks between them in the `ds_asio::DsAsio` class.

The callback passed to `addIoSM` is called every time a response is emitted.  A `boost::function` object created
with the default constructor will not be called.

## Related Code in Other Packages

### ds_bus and ds_bus_device

One common design pattern is to use the I/O State Machine to manage a bus.  `ds_base` provides to classes to help with
this.  `DsBus` in `ds_bus.h` provides a class that can expose an I/O State Machine to other ROS processes by publishing
all received messages on a common `bus` topic.  Individual device drivers can add their required commands via the `cmd`
service.  Similarly, the `DsBusDevice` class in `ds_bus_device.h` provides a common base class for those individual 
device drivers.

### ds_lambda_ps

The Lambda DC Power Supply Driver in the `ds_lambda_ps` package in the `ds_components` repository provides a helpful,
minimalist example of how to use the state machine by itself without any of the `DsBus` / `DsBusDevice` overhead.  This
driver talks was originally written to monitor and control a Lambda GenH80-9.5 or similar power supply over a serial
port.  The I/O state machine is used to manage polling for status and configuring the power supply's voltage/current in
response to user input. These power supplies are designed to be combined on a bus and thus require addressing.  This 
driver demonstrates how to use `forceNext` and command-specific callbacks to simplify driver development.


