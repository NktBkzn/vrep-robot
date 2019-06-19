#!/usr/bin/env python2
import roslib
roslib.load_manifest('first_package')
import rospy
import cv2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, Int16
from geometry_msgs.msg import Vector3
from math import pi
import numpy as np


LINEAR_VELOCITY_MINIMUM_THRESHOLD = 0.2
ANGULAR_VELOCITY_MINIMUM_THRESHOLD = 0.4

class platform():
    def __init__(self, file_name, fps=20, **kwargs):
        self.speed = kwargs.get('speed', [1, 1])
        self.robot = kwargs.get('name', 'platform')
        self.flag_move = True
        self.flag_handmove = True

        self.fps = fps
        self.file_name = file_name
        self.bridge = CvBridge()

        self.platform_move = rospy.Publisher("/platformMove", Float32MultiArray, queue_size=10)
        self.hand_mover = rospy.Publisher('/handMove', Float32MultiArray, queue_size=10)
        self.gripper = rospy.Publisher("/eeMove", Int16, queue_size=1)

        # self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Define the codec and create VideoWriter object
        # self.video_writer = None
        # self.image_sub = rospy.Subscriber('/image', Image, self.camera)

    def get_rotation(self, msg):
        print('get_rotation')
        print(msg.x, msg.y, msg.z)
        rospy.sleep(3)

    def control_platform(self):
        rospy.sleep(1)

        if self.flag_move:
            self.hand_move([0, -45, 10, 10, 10])
            I_left = rospy.Subscriber("/distIRLeft", Vector3, self.distIR_left)
            rospy.sleep(0.5)
            I_right = rospy.Subscriber("/distIRRight", Vector3, self.distIR_right)
            rospy.sleep(0.5)

            self.set_speed([-2, -2])
            while (self.left_back_coord[1] >= 0.3 or self.left_back_coord[1] ==0) and \
                    (self.right_back_coord[1] >= 0.3 or self.right_back_coord[1] == 0):
                I_left = rospy.Subscriber("/distHCLeft", Vector3, self.distHC_left)
                rospy.sleep(0.5)
                I_right = rospy.Subscriber("/distHCRight", Vector3, self.distHC_right)
                rospy.sleep(0.5)
            self.stop_robot()

            self.hand_move([0, 77, -43, -8, -33])
            rospy.sleep(2)
            # self.hand_move([0, 77, 10, 10, -33])
            # rospy.sleep(6)
            self.grab_obj(1)
            self.hand_move([0, -45, -43, -8, -33])
            rospy.sleep(2)
            self.hand_move([90, 20, -43, -8, -33])
            rospy.sleep(5)
            self.grab_obj(0)
            rospy.sleep(1)
            self.hand_move([0, 20, -43, -8, -33])

            turn = 0
            while turn < 2:
                rospy.sleep(1)
                I_left = rospy.Subscriber("/distIRLeft", Vector3, self.distIR_left)
                rospy.sleep(0.5)
                I_right = rospy.Subscriber("/distIRRight", Vector3, self.distIR_right)
                rospy.sleep(0.5)

                self.set_speed([-3, -3])
                while (self.left_back_coord[1] == 0) and (self.right_back_coord[1] == 0):
                    I_left = rospy.Subscriber("/distHCLeft", Vector3, self.distHC_left)
                    rospy.sleep(0.5)
                    I_right = rospy.Subscriber("/distHCRight", Vector3, self.distHC_right)
                    rospy.sleep(0.5)
                self.stop_robot()
                self.rotate_90()
                turn += 1



            # sub_HC_left = rospy.Subscriber("/distHCLeft", Vector3, self.distHC_left)
            # rospy.sleep(0.5)
            # sub_HC_right = rospy.Subscriber("/distHCRight", Vector3, self.distHC_right)
            # rospy.sleep(0.5)
            #
            # count = 0
            # self.set_speed([1, 1])
            # self.hand_move([90, 10, 10, 10, 10])
            # self.hand_move([180, 10, 10, 10, 10])
            # while self.left_front_coord[1] == 0 and self.right_front_coord[1] == 0:
            #     sub_HC_left = rospy.Subscriber("/distHCLeft", Vector3, self.distHC_left)
            #     rospy.sleep(0.5)
            #     sub_HC_right = rospy.Subscriber("/distHCRight", Vector3, self.distHC_right)
            #     rospy.sleep(0.5)
            # self.stop_robot()
            # self.rotate_90()
        # coord, speed = 0, 1
        # turn = 0
        # while turn <= 2:
        #     # self.set_speed([speed,speed])
        #     self.hand_move([180*coord,10,10,10,10])
        #     # rospy.sleep(2)
        #     coord = (coord + 1) % 2
        #     # speed = -speed
        #     rospy.sleep(5)
        #     turn += 1

    def camera(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_image = cv2.flip(cv_image, flipCode=1)  # try flipcode = 1
        except CvBridgeError as e:
            print(e)
        if self.video_writer is None:
            rows, cols, _ = cv_image.shape
            print(rows, cols)
            self.video_writer = cv2.VideoWriter(self.file_name, self.fourcc,  self.fps, (rows, cols))

        self.video_writer.write(cv_image)

    def rotate_90(self):
        self.stop_robot()
        self.set_speed([-2, 2])
        rospy.sleep(1.98)
        self.stop_robot()
        rospy.sleep(0.5)

    def clean_shutdown(self):
        if self.video_writer is not None:
            self.video_writer.release()
        print('Saving video file', self.file_name)

    def set_speed(self, speed):
        msg = Float32MultiArray()
        msg.data = speed
        self.platform_move.publish(msg)
        rospy.sleep(0.5)

    def stop_robot(self):
        rospy.loginfo("Stopping robot")
        self.set_speed([0, 0])

    def hand_move(self, speed):
        print('hand moving')
        msg2 = Float32MultiArray()
        msg2.data = speed
        self.hand_mover.publish(msg2)
        rospy.sleep(0.5)

    def grab_obj(self, grab):
        print('grabbing an object')
        msg = Int16()
        msg.data = grab
        self.gripper.publish(msg)
        rospy.sleep(0.5)

    def distHC_left(self, left_front_HC):
        self.left_front_coord = [left_front_HC.x, left_front_HC.y, left_front_HC.z]
        rospy.sleep(1)

    def distHC_right(self, right_front_HC):
        self.right_front_coord = [right_front_HC.x, right_front_HC.y, right_front_HC.z]
        rospy.sleep(1)

    def distIR_left(self, left_front_IR):
        self.left_back_coord = [left_front_IR.x, left_front_IR.y, left_front_IR.z]
        print(self.left_back_coord)

    def distIR_right(self, right_front_IR):
        self.right_back_coord = [right_front_IR.x, right_front_IR.y, right_front_IR.z]
        print(self.right_back_coord)

    def initialize(self):
        rospy.init_node("move", anonymous=True)
        self.control_platform()
        rospy.spin()
        rospy.on_shutdown(self.stop_robot)


if __name__ == "__main__":
    platform = platform('images/video2.avi')
    platform.initialize()
