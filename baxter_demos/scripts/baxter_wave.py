#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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

"""
Baxter Wave Demo 
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def main():
    """Baxter Wave Demo
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("baxter_wave_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    pos1 = {
        'left':  [-0.00728640873413086, 0.03374757729492188, -1.5569904979248048, 1.9075051075561524, 1.6390584699829103, 1.5075196175720216, -0.21360682446899415],
        'right':  [0.20133497817993165, 0.06442719301757813, 1.6003254551330568, 1.7406846970642091, -1.7146070236999513, 1.5481701084045412, 0.7742768018005372],    
    }
    pos2 = {
        'left': [-0.013038836682128907, 0.3236699458740235, -1.8158497555847168, 1.7916895582031251, 1.833107039428711, -1.3123205625366212, -0.10354370306396485],
        'right': [0.22472818516845705, 0.37160684544067385, 1.840393448162842, 1.7364662499023438, -1.8672381119201662, -1.0293011074951173, 0.4256796681518555],
    }

    pos3 = {
        'left': [0.03834951965332031, 0.4375680192443848, -2.2645391355285645, 1.2778059948486329, 1.7188254708618165, -1.2597817206115725, -0.005368932751464844],
        'right': [0.07478156332397462, 0.38848063408813477, 2.2511168036499023, 1.287393374761963, -1.6371409940002442, -1.2620826917907715, -0.466330158984375],
    }

    posOut = {
        'left': [0.6856894114013672, 0.7056311616210938, -1.3871021258605958, 1.9957090027587892, 2.1655973748229984, 1.6425099267517091, 0.5322913327880859],
        'right': [-0.5951845450195313, 0.6860729065979004, 1.4093448472595216, 2.017951724157715, -2.2346265101989746, 1.7222769276306154, 0.5384272559326172],
    }

    if limb == 'left':
        otherArm = 'right'
    else:
        otherArm = 'left'

    traj = Trajectory(limb)
    trajOther = Trajectory(otherArm)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

    p1 = pos1[limb]
    p2 = pos2[limb]
    p3 = pos3[limb]
    pOut = posOut[otherArm]
    traj.add_point(p1, 7.0)
    trajOther.add_point(pOut, 7.0)
    traj.add_point(p2, 10.0)
    traj.add_point(p3, 12.0)
    traj.add_point(p2, 14.0)
    traj.add_point(p3, 16.0)
    traj.add_point(p1, 19.0)
    #traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)
    trajOther.start()
    traj.start()
    traj.wait(22.0)
    trajOther.wait(9.0)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
