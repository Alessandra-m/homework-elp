#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import rospy
from sensor_msgs.msg import Image

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"

IMAGE = Image()


def cam_callback(msg):
  global IMAGE
  IMAGE.height = msg.height
  IMAGE.width = msg.width
  IMAGE.encoding = msg.encoding
  IMAGE.step = msg.step
  IMAGE.data = msg.data

def generate_image(width, height):
  random_image = np.zeros((width, height), dtype=np.uint8)
  
  return random_image

def main() -> None:
  global IMAGE
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)
  subscriber = rospy.Subscriber("pylon_camera_node/image_raw", Image, cam_callback)
  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)

  
  while not rospy.is_shutdown():
    
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    # publisher.publish(Image(width=0, height=0, encoding='mono8'))
    # data = generate_image(width = 320, height = 240)
    # IMAGE.data = data
    # IMAGE.width = 320
    # IMAGE.height = 240
    publisher.publish(IMAGE)
    
    rate.sleep()


if __name__ == '__main__':
    main()
