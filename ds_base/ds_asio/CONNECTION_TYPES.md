# Introduction

DsConnection has several subclasses to support different I/O methods.  Each connection has a name, and all of its
parameters, topics, etc. are defined in the namespace `node_namespace/connection_name/`.  

## Common Behaviors

All DsConnections can publish their raw I/O using the `ds_core_msgs::RawData` ROS type.  These messages are published
on the topic `node_namespace/connection_name/raw`.  

The following parameters are common to all connections:

* `publish_raw` Defaults to "true".  Set to "false" to disable publishing raw data on the raw data topic.  This is
not recommended, but may be a useful optimization for certain high-throughput connections.

## UDP Connection

Type String: "UDP"

The DsUdp connection class connects to data via User Datagram Packets (UDP) over an IP link.  Each UDP message 
generates a single callback.  Options are as follows.

* `buffer_size`  Size of the input buffer to use.  Messages longer than this value may be dropped or truncated.
Defaults to 512 bytes.  

* `udp_rx`  UDP receive port.  It is strongly recommended to use ports in the "registered" range (between 1024 
and 49151) when configuring new systems.

* `udp_tx`  Port to transmit UDP datagrams to.  It is strongly recommended to use ports in the "registered" range 
(between 1024 and 49151) when configuring new systems.

* `udp_address` The IP address to use.  If a multicast address is specified, the connection will receive on that 
multicast address.  If a hostname, host IP address, or broadcast address is specified then all outgoing traffic will
be directed to that IP.

## Serial Connection

Type string: "SERIAL"

The DsSerial connection class provides interfacing to an RS232-style serial link through a file descriptor.  Standard
options are provided for the usual RS232 arguments, including baudrate, parity, etc.  Unlike UDP connections, serial 
file descriptors do not provide a straightforward way to break up messages.  Instead, a matcher function is used
to generate one callback with a complete packet or message of serial data.  Several standard matcher functions are
provided (see "Matcher Functions").  Matcher functions define their own additional parameters.  Drivers may also 
specify their own matcher function via the C++ API.

The options are as follows:

* `port` The file descriptor to use.  Probably "/dev/ttySOMETHING"

* `baud` The baudrate to use.  Defaults to 9600 and must be a valid baudrate.  Probably one of 9600, 19200, 38400,
 57600, or 115200.
 
* `data_bits` Number of data bits in the serial character.  Defaults to 8, which is almost always the right number.

* `stop_bits`  Sets the number of stop bits on the serial port.  Either "1" or "2".

* `parity` A string specifying parity options.  One of "none", "even", or "odd".

* `matcher` The matcher function to use.  Matcher functions are described separately.

Hardware flow control is  not currently supported.


## TCP Client Connection

Type string: "TCPCLIENT"

The TCP Client Connection connects to a TCP server and presents any data coming in on the socket via a callback.
No matcher function is employed.  TCP connects frequently cannot detect when the connection is lost, so this 
connection class includes a timeout.  If no data is received before the timeout expires, the socket is closed and
re-opened.  Up to five attempts to re-open the socket will be made 10 seconds apart.

Available options are:

* `tcp_address` The address of the server to connect to.

* `tcp_port` The port of the server to connect to.

* `timeout_sec` The period of the no-data timeout to initation a close/reconnect.  Defaults to 30 seconds.

* `buffer_size` The maximum buffer size of a single message, in bytes.  Messages bigger than this may be dropped or
 truncated. The largest possible size permitted by TCP is 65535 bytes.  Defaults to 512 bytes.

## ROS Raw Connection

Type string "ROSRAW"

The RosRaw connection allows the user to replay previously-recorded raw data into a DsConnection for testing.  Although
primarily used as an input, the connection does support an output topic as well.  Options are:

* `topic_rx` The topic to listen on.  Any message with the data direction set to "in" will trigger a callback in the 
driver.

* `topic_tx` The topic to transmit outgoing data to.  This is redundant with the raw I/O topic, but may be useful for
testing or something.

# Matcher Functions

TODO