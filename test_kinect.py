import turtle_bot
from turtle_bot import get_robot
import cv2
import numpy as np
import os
import time
import math
import random
import thread
import rospy


if __name__ == "__main__":
	robot = get_robot()
	r = rospy.Rate(100)
	while True:
		robot.move_searh_n_destroy(0.20, 1.0)
		r.sleep()

		#if robot.current_mask != None:
			#cv2.imshow("Image",robot.horrible)

		
