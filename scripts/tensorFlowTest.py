#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: October 20, 2016
# 

import sys
import rospy
import cv2
import tensorflow as tf
import os
import threading
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from do8autonomouscar.msg import objectMap



def main(args):
    hello = tf.constant('Hellow, TensorFlow!')
    sess = tf.Session()
    print(sess.run(hello))

    a = tf.constant(10)
    b = tf.constant(32)
    print(sess.run(a + b))

if __name__ == '__main__':
    main(sys.argv)
