#!/usr/bin/env python3
import os
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range

# For NN
import argparse
import torch
from network import ResnetUnetHybrid
import image_utils

class Example(object):

    def __init__(self):
        rospy.loginfo("[Example] loaging")
        rospy.on_shutdown(self.shutdown)

        self.gui = os.getenv('GUI')=='true' or os.getenv('GUI')=='True'

        sub_image_topic_name = "/head/camera1/image_raw"
        self.camera_subscriber = rospy.Subscriber(sub_image_topic_name, Image, self.camera_cb)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.odometry = Odometry()
        self.command = Twist()

        self.sonar_data = [0, 0, 0, 0]

        self.curent_image = None
        self.bridge = CvBridge()

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(self.device)
        self.cnt = 0

    def __del__(self):
        pass

    def shutdown(self):
        # stop robots here
        self.cmd_vel.publish(Twist())

    def predict_img(self, img):
        # load model
        rospy.loginfo(f'Loading model..., {self.cnt}')
        self.cnt += 1
        model = ResnetUnetHybrid.load_pretrained(device=self.device)
        model.eval()

        # load image
        img = img[..., ::-1]
        img = image_utils.scale_image(img)
        img = image_utils.center_crop(img)
        inp = image_utils.img_transform(img)
        inp = inp[None, :, :, :].to(self.device)

        # inference
        output = model(inp)
        # transform and plot the results
        output = output.cpu()[0].data.numpy()

        depth = np.transpose(output, (1, 2, 0))[:, :, 0]

        return depth

    def camera_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        # some processing here

        #display from onbourd camera
        if self.gui != False:
            cv.imshow("output", frame)

            depth = self.predict_img(frame)
            cv.imshow("d", depth)

            cv.waitKey(1)

    def range_front_callback(self, msg):
        self.sonar_data[0] = msg.range

    def odom_callback(self, msg: Odometry):
        self.odometry = msg
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = msg.pose.pose.position.z

    def spin(self):

        rate = rospy.Rate(30)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - t0

            self.command.linear.x = 0.0
            self.command.linear.y = 0.0
            self.command.angular.z = 0.5

            self.cmd_vel.publish(self.command)

            rate.sleep()


def main(args=None):
    rospy.init_node("example_node")

    exp = Example()
    exp.spin()


if __name__ == "__main__":
    main()
