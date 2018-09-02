#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rosbag
import rospy
from sensor_msgs.msg import Image
from time import sleep
import argparse
import cv2
from cv_bridge import CvBridge, CvBridgeError


def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', help='path for d435 rosbag', required=True)
    parser.add_argument('--dump', default=False, action='store_true',
                        help='dump image while play rosbag.')
    args = parser.parse_args()
    return args


class D435Publisher:
    def __init__(self):
        self.image_topic = '/device_0/sensor_1/Color_0/image/data'
        self.depth_topic = '/device_0/sensor_0/Depth_0/image/data'
        self.infrared_topic = '/device_0/sensor_0/Infrared_1/image/data'

        self.image_pub = rospy.Publisher(
            self.image_topic, Image, queue_size=10)
        self.depth_pub = rospy.Publisher(
            self.depth_topic, Image, queue_size=10)
        self.infrared_pub = rospy.Publisher(
            self.infrared_topic, Image, queue_size=10)
        self.bridge = CvBridge()

    def read(self, path, dump):
        r = rospy.Rate(10)
        directory = os.path.dirname(path)
        index = 0
        with rosbag.Bag(path) as bag:
            for topic, msg, t in bag.read_messages(
                    topics=[self.image_topic,
                            self.depth_topic,
                            self.infrared_topic]):
                if topic == self.image_topic:
                    self.image_pub.publish(msg)

                    if dump:
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print(e)
                        cv2.imwrite(
                            os.path.join(directory, "{:06d}.png").format(index), cv_image)
                    index += 1
                if topic == self.depth_topic:
                    self.depth_pub.publish(msg)
                if topic == self.infrared_topic:
                    self.infrared_pub.publish(msg)
                rospy.sleep(0.1)


def main():
    options = parseArguments()

    rospy.init_node("d435_publisher", anonymous=True, disable_signals=True)
    publisher = D435Publisher()

    publisher.read(options.path, options.dump)


if __name__ == "__main__":
    main()
