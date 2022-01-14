# Copyright 2018 Woods Hole Oceanographic Institution
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import rospy

import threading
from ds_core_msgs.msg import ParamUpdate, ParamDescription, KeyBool, KeyDouble, KeyFloat, KeyInt, KeyString

__all__ = ['BoolParam', 'IntParam', 'FloatParam', 'DoubleParam', 'StringParam', 'EnumParam', 'ParamConnection']


class _UpdatingParam(object):
    """ Abstract base class for a single auto-updating parameter."""

    def __init__(self, connection, name, advertise):
        self._conn = connection
        self._name = name
        self._advertise = advertise

        self._dirty = False

        self._last = None
        self._value = None

    def _load_from_server(self):
        """ Private inner function to load from the parameter server.  Not for use by the user"""
        self._value = self.cast(rospy.get_param(self._name))

    def _set_on_server(self):
        """ Private inner function to synch a parameter to the server.  Not used by the user"""
        rospy.set_param(self._name, self._value)

    def _update_value(self, v):
        """ Private inner function used by a new-message callback """
        self._last = self._value
        self._value = v
        self._dirty = False

    def set(self, value):
        """ Set an updating parameter.  Also updates the parameter server and publishes an update """
        if self._conn.is_locked():
            if not self._dirty:
                # Only update the previous value if we're currently sync'd to the system.
                # If we're locked, but have already been changed, then don't update the previous
                # to implement this "previous is the last thing that went out" metaphor
                self._last = self._value

            self._value = self.cast(value)

            # we're locked, so flag dirty and return
            self._dirty = True
            return
        else:
            # Update local cache and stuff
            self._last = self._value
            self._value = self.cast(value)

            # Now the parameter server
            self._set_on_server()
            self._conn._signal_update(self)

    def get(self):
        """ Get the current value.  Grabs from local cache, super-fast"""
        return self._value

    def get_previous(self):
        return self._last

    def yaml_description(self):
        return '{ name: \"' + self._name + '\", type: \"' + self.type() + '\" }'

    def name(self):
        return self._name


class BoolParam(_UpdatingParam):
    def __init__(self, connection, varname, advertise):
        super(BoolParam, self).__init__(connection, varname, advertise)

    @staticmethod
    def type():
        return 'bool'

    @staticmethod
    def cast(var):
        return bool(var)

    def _fill_update_message(self, msg):
        val = KeyBool()
        val.key = self._name
        val.value = self._value
        msg.bools.append(val)
        return msg


class IntParam(_UpdatingParam):
    def __init__(self, connection, varname, advertise):
        super(IntParam, self).__init__(connection, varname, advertise)

    @staticmethod
    def type():
        return 'int'

    @staticmethod
    def cast(var):
        return int(var)

    def _fill_update_message(self, msg):
        val = KeyInt()
        val.key = self._name
        val.value = self._value
        msg.ints.append(val)
        return msg


class FloatParam(_UpdatingParam):
    def __init__(self, connection, varname, advertise):
        super(FloatParam, self).__init__(connection, varname, advertise)

    @staticmethod
    def type():
        return 'float'

    @staticmethod
    def cast(var):
        return float(var)

    def _fill_update_message(self, msg):
        val = KeyFloat()
        val.key = self._name
        val.value = self._value
        msg.floats.append(val)
        return msg


class DoubleParam(_UpdatingParam):
    def __init__(self, connection, varname, advertise):
        super(DoubleParam, self).__init__(connection, varname, advertise)

    @staticmethod
    def type():
        return 'double'

    @staticmethod
    def cast(var):
        return float(var)

    def _fill_update_message(self, msg):
        val = KeyDouble()
        val.key = self._name
        val.value = self._value
        msg.doubles.append(val)
        return msg


class StringParam(_UpdatingParam):
    def __init__(self, connection, varname, advertise):
        super(StringParam, self).__init__(connection, varname, advertise)

    @staticmethod
    def type():
        return 'string'

    @staticmethod
    def cast(var):
        return str(var)

    def _fill_update_message(self, msg):
        val = KeyString()
        val.key = self._name
        val.value = self._value
        msg.strings.append(val)
        return msg


class EnumParam(IntParam):
    def __init__(self, connection, varname, advertise):
        super(EnumParam, self).__init__(connection, varname, advertise)
        self._named_values = {}

    @staticmethod
    def type():
        return 'enum'

    # inherit cast and _fill_update_message from int

    def has_named_value(self, name):
        return name in self._named_values

    def set_value_by_name(self, name):
        self.set(self._named_values[name])

    def get_value_by_name(self):
        v = self.get()
        if v in self._named_values.values():
            for name, value in self._named_values.iteritems():
                if value == v:
                    return name
        else:
            return str(self.get())

    def add_named_value(self, name_str, value_int):
        self._named_values[name_str] = value_int
        self._conn._publish_description()

    def names(self):
        return self._named_values.keys()

    def values(self):
        return self._named_values.values()

    def yaml_description(self):
        ret = '{ name: \"%s\",' % self._name
        ret += ' type: \"%s\",' % self.type()
        ret += ' enum: {'
        for name, value in self._named_values.iteritems():
            ret += (' \"%s\": \"%s\", ' % (name, value))

        ret += '} }'

        return ret


class ParamConnection(object):
    def __init__(self):
        self._locked = False
        self._params = {}
        self._callback = None
        self._mutex = threading.Lock()

        self._mutex.acquire()

        try:

            # Build our name the same way the C++ system works
            self._conn_name = rospy.resolve_name(rospy.get_namespace() + "/" + rospy.get_name()) + "##"
            self._conn_name += ('%09d' % rospy.get_rostime().nsecs)
    
            # Setup our I/O
            self._descriptionPublisher = rospy.Publisher("/updating_param/description", ParamDescription,
                                                         queue_size=1, latch=True)
    
            self._updatePublisher = rospy.Publisher("/updating_param/updates", ParamUpdate,
                                                    queue_size=1, latch=True)
    
            self._updateSubscriber = rospy.Subscriber("/updating_param/updates", ParamUpdate,
                                                      callback=self._update_callback,
                                                      queue_size=10)
        except:
            self._mutex.release()
            raise
	self._mutex.release()

    def connect(self, varname, vartype, advertise, caller_id=None):
        '''Connect a variable via this connection
        @varname : The variable on the parameter server to connect to
        @vartype : One of the following: BoolParam, IntParam, FloatParam, DoubleParam, StringParam or EnumParam
        @advertise : A flag indicating whether to advertise this variable on param_description
        @caller_id (default=None): The same as the caller_id parameter to resolve_name.  You can use this
        to resolve names relative to some other namespace'''

        self._mutex.acquire()
        try:
            fullname = rospy.resolve_name(varname, caller_id)

            # First, check to see if we already have a copy in our local cache
            if fullname in self._params:
                if advertise:
                    # Force advertisement to happen if its requested
                    self._params[fullname]._advertise = True
                    self._publish_description()

                self._mutex.release()
                return self._params[fullname]

            if not rospy.has_param(fullname):
                 raise KeyError("Variable named \"" + fullname + "\" does not exist on the parameter server!")

            p = vartype(self, fullname, advertise)
            p._load_from_server()

            self._params[fullname] = p

            self._publish_description()
        except:
            self._mutex.release()
            raise
	self._mutex.release()

        return p

    def name(self):
        return self._conn_name

    def set_callback(self, callback):
        self._mutex.acquire()
        try:
            self._callback = callback
        except:
            self._mutex.release()
            raise
	self._mutex.release()

    def lock(self):
        self._mutex.acquire()
        try:
            self._locked = True
        except:
            self._mutex.release()
            raise
	self._mutex.release()
  

    def unlock(self):
        self._mutex.acquire()
        try:
            if self._locked:
                # we have to send any pending updates
                msg = ParamUpdate()

                any_set = False
                for p in self._params.values():
                    if p._dirty:
                         msg = p._fill_update_message(msg)
                         p._set_on_server()
                         p._dirty=False
                         any_set = True
                msg.stamp = rospy.get_rostime()
                msg.source = self._conn_name

                if any_set:
                    self._updatePublisher.publish(msg)

            self._locked = False
        except:
            self._mutex.release()
            raise
        self._mutex.release()

    def is_locked(self):
        return self._locked

    def _signal_update(self, param):
        #if self._mutex.locked():
            #print '_signal_update: MUTEX ALREADY LOCKED!'
        self._mutex.acquire()
        try:
            msg = ParamUpdate()
            msg = param._fill_update_message(msg)

            msg.stamp = rospy.get_rostime()
            msg.source = self._conn_name

            self._updatePublisher.publish(msg)
        except:
            self._mutex.release()
            raise
        self._mutex.release()


    def _publish_description(self):
        # Build our yaml description manually
        payload = """node:
            name: %s
            namespace: %s
        params: [""" % (rospy.get_name(), rospy.get_namespace())
        for p in self._params.values():
            if p._advertise:
                payload += p.yaml_description()
                payload += ', '
        payload += ']\n'

        # Build the message
        msg = ParamDescription()
        msg.yaml_payload = payload

        # Publish!
        self._descriptionPublisher.publish(msg)

    def _update_callback(self, msg):
        if msg.source == self._conn_name:
            # Don't respond to messages from ourself
            #print 'Ignoring callback from our own message!'
            return
        
        #if self._mutex.locked():
        #    print '_update_callback: MUTEX ALREADY LOCKED!'
        self._mutex.acquire()
        try:


            updated_params = []
            updated_params.extend(self._update_params(msg.bools))
            updated_params.extend(self._update_params(msg.ints))
            updated_params.extend(self._update_params(msg.floats))
            updated_params.extend(self._update_params(msg.doubles))
            updated_params.extend(self._update_params(msg.strings))

            self._mutex.release()

            if self._callback is not None:
                #print msg
                self._callback(updated_params)
        except:
            self._mutex.release()
            raise

    def _update_params(self, keyvalue):
        ret = []
        for kv in keyvalue:
            if kv.key in self._params:
                p = self._params[kv.key]
                p._update_value(kv.value)

                ret.append(p)
        return ret
