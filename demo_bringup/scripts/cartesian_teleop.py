#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import TwistStamped

import sys
import select
import termios
import tty

msg = """
Control Your Arm Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'q': (1, 0, 0, 0, 0, 0),
    'w': (0, 1, 0, 0, 0, 0),
    'e': (0, 0, 1, 0, 0, 0),
    'a': (-1, 0, 0, 0, 0, 0),
    's': (0, -1, 0, 0, 0, 0),
    'd': (0, 0, -1, 0, 0, 0),
    'r': (0, 0, 0, 1, 0, 0),
    't': (0, 0, 0, 0, 1, 0),
    'y': (0, 0, 0, 0, 0, 1),
    'f': (0, 0, 0, -1, 0, 0),
    'g': (0, 0, 0, 0, -1, 0),
    'h': (0, 0, 0, 0, 0, -1),
}

speedBindings = {
    'u': (1.1, 1.1,1.1, 1.1,1.1,1.1),
    'j': (.9, .9,.9, .9,.9,.9),
    'i': (1.1, 1.1,1.1, 1.1,1.1,1.1),
    'k': (.9, .9,.9, .9,.9,.9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed_x = 0.2
speed_y = 0.2
speed_z = 0.2
turn_x = 0.2
turn_y = 0.2
turn_z = 0.2


def vels(speed_x, speed_y, speed_z, turn_x, turn_y, turn_z):
    return "currently:\tspeed_x %s\tturn_x %s\tspeed_y %s\tturn_y %s\tspeed_z %s\tturn_z %s " % (speed_x, turn_x, speed_y, turn_y, speed_z, turn_z)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('manipulator_cartesian_teleop')
    pub = rospy.Publisher('manipulator_cartesian_command', TwistStamped, queue_size=5)

    x = 0
    y = 0
    z = 0
    th_x = 0
    th_y = 0
    th_z = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed_x = 0
    target_turn_x = 0
    control_speed_x = 0
    control_turn_x = 0
    target_speed_y = 0
    target_turn_y = 0
    control_speed_y = 0
    control_turn_y = 0
    target_speed_z = 0
    target_turn_z = 0
    control_speed_z = 0
    control_turn_z = 0
    try:
        print(msg)
        print(vels(speed_x, turn_x, speed_y, turn_y, speed_z, turn_z))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th_x = moveBindings[key][3]
                th_y = moveBindings[key][4]
                th_z = moveBindings[key][5]
                count = 0
            elif key in speedBindings.keys():
                speed_x = speed_x * speedBindings[key][0]
                speed_y = speed_y * speedBindings[key][1]
                speed_z = speed_z * speedBindings[key][2]
                turn_x = turn_x * speedBindings[key][3]
                turn_y = turn_y * speedBindings[key][4]
                turn_z = turn_z * speedBindings[key][5]
                count = 0
                print(vels(speed_x,turn_x,speed_y,turn_y,speed_z,turn_z))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'l':
                x=0
                y=0
                z=0
                th_x=0
                th_y=0
                th_z=0
                control_speed_x=0
                control_turn_x=0
                control_speed_y=0
                control_turn_y=0
                control_speed_z=0
                control_turn_z=0
            else:
                count=count + 1
                if count > 200:
                    x=0
                    y=0
                    z=0
                    th_x=0
                    th_y=0
                    th_z=0
                if (key == '\x03'):
                    break

            target_speed_x=speed_x * x
            target_speed_y=speed_y * y
            target_speed_z=speed_z * z
            target_turn_x=turn_x * th_x
            target_turn_y=turn_y * th_y
            target_turn_z=turn_z * th_z

            if target_speed_x > control_speed_x:
                control_speed_x=min(target_speed_x, control_speed_x + 0.02)
            elif target_speed_x < control_speed_x:
                control_speed_x=max(target_speed_x, control_speed_x - 0.02)
            else:
                control_speed_x=target_speed_x

            if target_speed_y > control_speed_y:
                control_speed_y=min(target_speed_y, control_speed_y + 0.02)
            elif target_speed_y < control_speed_y:
                control_speed_y=max(target_speed_y, control_speed_y - 0.02)
            else:
                control_speed_y=target_speed_y

            if target_speed_z > control_speed_z:
                control_speed_z=min(target_speed_z, control_speed_z + 0.02)
            elif target_speed_z < control_speed_z:
                control_speed_z=max(target_speed_z, control_speed_z - 0.02)
            else:
                control_speed_z=target_speed_z

            if target_turn_x > control_turn_x:
                control_turn_x=min(target_turn_x, control_turn_x + 0.1)
            elif target_turn_x < control_turn_x:
                control_turn_x=max(target_turn_x, control_turn_x - 0.1)
            else:
                control_turn_x=target_turn_x

            if target_turn_y > control_turn_y:
                control_turn_y=min(target_turn_y, control_turn_y + 0.1)
            elif target_turn_y < control_turn_y:
                control_turn_y=max(target_turn_y, control_turn_y - 0.1)
            else:
                control_turn_y=target_turn_y

            if target_turn_z > control_turn_z:
                control_turn_z=min(target_turn_z, control_turn_z + 0.1)
            elif target_turn_z < control_turn_z:
                control_turn_z=max(target_turn_z, control_turn_z - 0.1)
            else:
                control_turn_z=target_turn_z

            twist=TwistStamped()
            twist.header.stamp = rospy.get_rostime()
            twist.twist.linear.x=control_speed_x
            twist.twist.linear.y=control_speed_y
            twist.twist.linear.z=control_speed_z
            twist.twist.angular.x=control_turn_x
            twist.twist.angular.y=control_turn_y
            twist.twist.angular.z=control_turn_z
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist=TwistStamped()
        twist.twist.linear.x=0
        twist.twist.linear.y=0
        twist.twist.linear.z=0
        twist.twist.angular.x=0
        twist.twist.angular.y=0
        twist.twist.angular.z=0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
