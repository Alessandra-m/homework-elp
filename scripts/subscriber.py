#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC: Final[str] = "image"


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
  # конвертация ROS-сообщения в OpenCV изображение
  # Q: Каков формат у изображения?
  image = cv_bridge.imgmsg_to_cv2(msg)
  
  # Задание 2: Реализуйте отображение полученного изображения.
  # Используйте функции cv2.imshow и cv2.waitKey
  cv2.imshow(image)
  # cv2.waitKey()


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
  rospy.loginfo(f"OpenCV version: {cv2.__version__}")

  # wait for the images to arrive or throw an exception
  sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=3.0)
  
  if sample is not None:
    rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
  
  cv_bridge: CvBridge = CvBridge()

  rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size=None)

  # Q: Что происходит в данной строчке кода?
  rospy.spin()


if __name__ == '__main__':
  main()