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
import cv2

from sensor_msgs.msg import (
    Image,
)
import baxter_interface

from cv_bridge import CvBridge, CvBridgeError

from time import sleep

from baxter_interface import CHECK_VERSION

# import sys

class image_capturer:
    def __init__(self, limb):
        self.cam_sub = rospy.Subscriber("/cameras/%s_hand_camera/image" % limb, Image, self.callback, queue_size=None)
        self.bridge = CvBridge()
        self.writeImage = False

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
            cv2.imwrite("/home/demo/Desktop/selfie_%i.jpg" % now.secs, cv_image)
            print("Image Saved as /home/demo/Desktop/selfie_%i.jpg" % now.secs)
            self.writeImage = False
            print("\nPress Enter to Capture Image:\n")

def main():
    """Selfie Demo for The UNR Baxter Robot
        The selfie image will be stored on the desktop 
        in a folder called "selfie_images" 
    """
    rospy.init_node("image_capturer")
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
    ic = image_capturer(limb)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
        # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #    print "Shutting down"
    print("Press Enter to Capture Image:")
    while not rospy.is_shutdown():
        r = rospy.Rate(10)
        r.sleep()
        char = sys.stdin.read(1)

        if char != None:
            print("Prepping Image Capture . . .")
            ic.writeImage = True


if __name__ == "__main__":
    main()
