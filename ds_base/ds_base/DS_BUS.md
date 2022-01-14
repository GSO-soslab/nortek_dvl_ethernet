# DS Bus

The bus node is intended to provide a way for many ROS device nodes to share a single bus connected to a single 
DsConnection.

In order for the DsBus node to work unmodified, the bus protocol must follow certain rules.
* The DsBus node must function as the bus master device
* All slaves on the bus must only reply when properly addressed
* A standard DsConnection, configured only via the parameter server, must be able to properly break up bus traffic

The DsBus node uses an Io State Machine to control the bus.  The following ROS interface is exposed.

## The ```bus``` Topic

All traffic received on the bus is sent to the node's bus topic as a ```ds_core_msgs::RawData``` message.

## The ```cmd``` Service

The I/O state machine can be directly controlled via the ```cmd``` service.  The ```IoSMcommand``` service class
includes options for manipulating regular commands, adding preempt commands, etc.  Each ```IoSMcommand``` request is 
processed at once.  Although every command is attempted and some commands may succeed or fail, it is guaranteed that
commands are added to the state machine in the order they appear in the message.  Command sequences protected via the 
```force_next``` parameter will work as intended.

## The ```preempt_cmd``` Topic

The node also listens to the ```preempt_cmd``` topic and will add all commands received over that topic directly
to the I/O state machine.  Again, all commands are processed in the order they appear in the ```IoCommandList```, so 
commands that must happen together will work as expected.

