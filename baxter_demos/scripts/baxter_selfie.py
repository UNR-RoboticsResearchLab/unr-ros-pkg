#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,#!/usr/bin/env python

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
#SELFIE!!!!
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib
import cv
import cv2
import cv_bridge

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import (
    Image,
)
import baxter_interface

from cv_bridge import CvBridge, CvBridgeError

from time import sleep

from baxter_interface import CHECK_VERSION

from time import sleep

from baxter_interface.camera import CameraController

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



class image_capturer:    
    def __init__(self, limb):
        self.cam_sub = rospy.Subscriber("/cameras/%s_hand_camera/image" % limb, Image, self.callback, queue_size=None)
        self.bridge = CvBridge()
        self.writeImage = False
        self._head = baxter_interface.Head()

    def callback(self, data):
        # print("Taking Image . . .")
        # bridge = CvBridge()

        if self.writeImage == True:
            print("Taking Image . . .")
            try: 
                print("Converting Image . . .")
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError, e:
                print e
            #Write image to file
            now = rospy.get_rostime()
            print("Writing Image . . .")
            cv2.imwrite("/home/demo/Desktop/selfie_images/selfie_%i.jpg" % now.secs, cv_image)
            print("Image Saved as /home/demo/Desktop/selfie_images/selfie_%i.jpg" % now.secs)
            self.writeImage = False
            print("\nPress Enter to Capture Image:\n")

    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)

    def set_head(self, angle):
        self._head.set_pan(angle, speed=30, timeout=0)
        rospy.sleep(1)

    def reset_head(self):
        #Reset head to 0 position to cleanup after demo is finished
        self._head.set_pan(0, speed=30, timeout=0)
        rospy.sleep(1)


def open_camera(camera, res, *_args, **_kwds):
    cam = CameraController(camera)
    cam.close()
    cam.resolution = res
    cam.open()


def main():
    """Selfie Demo for The UNR Baxter Robot
        The selfie image will be stored on the desktop 
        in a folder called "selfie_images" 
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

    rospy.init_node("image_capturer")

    print("Initializing node... ")
    ic = image_capturer(limb)


    #Opening Camera)
    open_camera("%s_hand_camera" %limb, (1280, 800)) 
    # open_camera("%s_hand_camera" %limb, (640,400)) 


    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    #Uncomment which face is desired for the selfie 
    print("Publishing Face. . .")
    ic.send_image("/home/demo/ros_ws/src/baxter_examples/share/images/Baxter_Face_%s.jpg" %limb)
    # ic.send_image("/home/demo/ros_ws/src/baxter_examples/share/images/Baxter_Face_left.jpg")
    # ic.send_image("/home/demo/ros_ws/src/baxter_examples/share/images/Baxter_Angry_Face.jpg")
    # ic.send_image("/home/demo/ros_ws/src/baxter_examples/share/images/Baxter_Face_Unamused.jpg")


    arm_positions_left = {
        # 'left':  [0.18100973276367188, -0.6373690166381837, -1.664752648150635, 0.9595049817260742, -0.6250971703491212, 2.0946507634643554, 0.8724515721130371],
        # 'right':  [-0.22779614674072268, 0.3666214078857422, 1.3050341538024903, 1.0757040262756348, -2.970937287542725, -1.5711798201965332, -2.862024651727295], 
        'left':  [0.18, -0.64, -1.66, 0.96, -0.63, 2.09, 0.87],
        'right':  [-0.23, 0.37, 1.31, 1.08, -2.97, -1.57, -2.86],    
    }
    arm_positions_right = {
        # 'left':  [0.14342720350341798, 0.5215534672851563, -1.4028254289184572, 1.3556555197448732, -0.2949078061340332, 1.389019601843262, 1.6076118638671877],
        # 'right':  [-0.2684466375732422, -0.2185922620239258, 2.172116793164063, 1.4220001887451172, -3.0457188508666992, -1.5700293346069336, 2.10232066739502],    
        'left':  [0.14, 0.52, -1.40, 1.35, -0.29, 1.38, 1.60],
        'right':  [-0.27, -0.22, 2.17, 1.42, -3.05, -1.57, 2.10],    
    }

    #Setting limb positions based on left/right selfie position
    if limb == 'left':
        otherArm = 'right'
        p1 = arm_positions_left[limb]
        p2 = arm_positions_left[otherArm]
        # Setting Head Tilt
        print("Setting Head Position. . .")
        ic.set_head(.25)
    else:
        otherArm = 'left'
        p1 = arm_positions_right[limb]
        p2 = arm_positions_right[otherArm]
        # Setting Head Tilt
        print("Setting Head Position. . .")
        ic.set_head(-.25)

    # Position Arms correctly: 
    print("Moving arms into selfie position. . .")
    traj = Trajectory(limb)
    trajOther = Trajectory(otherArm)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)
    rospy.on_shutdown(ic.reset_head)

    traj.add_point(p1, 7.0)
    trajOther.add_point(p2, 7.0)
    #traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)
    trajOther.start()
    traj.start()
    traj.wait(10.0)
    trajOther.wait(10.0)

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print "Shutting down"

    print("Press Enter to Capture Image:")
    # rospy.spin()
    while not rospy.is_shutdown():
        # r = rospy.Rate(200)
        # r.sleep()
        # print(" . . .")
        char = sys.stdin.read(1)
        # sleep(5.0)

        if char != None:
            print("Prepping Image Capture . . .")
            ic.writeImage = True

if __name__ == "__main__":
    main()
