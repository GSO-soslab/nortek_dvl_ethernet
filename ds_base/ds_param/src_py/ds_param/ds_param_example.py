#!/usr/bin/env python
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
import sys

import rospy
from ds_param import *


class ParamDemo(object):
    def __init__(self):
        # Load some initial parameters
        self.other_node = rospy.get_param(rospy.get_name() + '/other_node')
        self.start_idx  = rospy.get_param(rospy.get_name() + '/start_idx')

        self.conn = ParamConnection()

        # Connect to OUR shared parameters
        self.param_enum = self.conn.connect(rospy.get_name() + '/test_enum_param', EnumParam, True)
        self.param_enum.add_named_value("Option 1", 1)
        self.param_enum.add_named_value("non-consecutive option 6", 6)
        self.param_int = self.conn.connect(rospy.get_name() + '/test_int_param', IntParam, True)
        self.param_str = self.conn.connect(rospy.get_name() + '/test_str_param', StringParam, True)

        # Connect to the OTHER NODE's shared parameters
        self.other_int = self.conn.connect('/' + self.other_node + "/test_int_param", IntParam, False)
        self.other_str = self.conn.connect('/' + self.other_node + "/test_str_param", StringParam, False)

        # Setup two parameters that update together
        self.param_bool_atomic = self.conn.connect(rospy.get_name() + "/test_atomic_bool", BoolParam, False)
        self.param_int_atomic  = self.conn.connect(rospy.get_name() + "/test_atomic_int",   IntParam, False)

        self.index = self.start_idx

    def _timer_callback(self, evt):
        str_to_send = 'Idx %d -> %d' % (self.start_idx, self.index)

        # Send our parameters
        self.other_int.set(self.index)
        self.other_str.set(str_to_send)

        # Read our own
        if self.param_int.get_previous is None:
            old_str = 'NOT SET'
        else:
            old_str = str(self.param_int.get())

        # Update some stuff atomically
        self.conn.lock()

        self.param_bool_atomic.set(not self.param_bool_atomic.get())
        self.param_int_atomic.set(self.index)

        self.conn.unlock()

        rospy.logerr("%s: MY param %d --> \"%s\" (was %s)    OTHER param: %d --> \"%s\"",
                     rospy.get_name(),
                     self.param_int.get(), self.param_str.get(), old_str,
                     self.other_int.get(), self.other_str.get())

        self.index += 1

    def setup_callback(self):
        self.conn.set_callback(self.change_callback)


    def change_callback(self, params):
        rospy.logerr('/------------------------------------------')
        for p in params:
            rospy.logerr("|           PARAM changed from \"%s\" to \"%s\"", p.name(), p.get())
        rospy.logerr('\------------------------------------------')


def main():

    rospy.init_node('ds_param_demo', sys.argv)

    demo = ParamDemo()

    timer = rospy.timer.Timer(rospy.Duration(3.0), demo._timer_callback, oneshot=False)

    use_callback = rospy.get_param(rospy.get_name() + "/use_callback")
    if use_callback:
        demo.setup_callback()

    # Spin until ctrl-C
    rospy.spin()


if __name__ == '__main__':

    main()
