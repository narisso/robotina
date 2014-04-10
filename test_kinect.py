import turtle_bot
from turtle_bot import get_robot
import cv2
import numpy as np
import os
import time
import math
import random
import thread

if __name__ == "__main__":
	robot = get_robot()
	while True:
		robot.move_searh_n_destroy(0.23, 1.0)
		if robot.current_mask != None:
			cv2.imshow("Image",robot.current_mask)

		k = cv2.waitKey(1)
		if k == 27:
			break
		
