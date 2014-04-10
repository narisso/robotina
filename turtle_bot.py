import sys
import time
import numpy as np
import cv2
from cv2 import cv
from cv_bridge import CvBridge, CvBridgeError 
import rospy
import pygame
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound
from kobuki_msgs.msg import WheelDropEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from tf import transformations as trans

_turtlebot_singleton = None

import pickle


def get_robot():
    global _turtlebot_singleton
    if _turtlebot_singleton is None:
        _turtlebot_singleton = Turtlebot()
    return _turtlebot_singleton


class Turtlebot(object):
    max_linear = 0.23
    min_linear = 0.05
    max_angular = 2.0
    d1 = 0.7
    d2 = 0.6
    deltaD = d1-d2
    speed_const = max_linear / deltaD
    v0 = speed_const * d2
    

    def __init__(self):

        pygame.init()
        rospy.init_node('pyturtlebot', anonymous=True)
        rospy.myargv(argv=sys.argv)

        self.__x = None
        self.__y = None
        self.__angle = None
        self.__cumulative_angle = 0.0
        self.__have_odom = False
        self.bridge = CvBridge()

        self.on_bumper = None
        self.current_state = None
        self.current_substate = None

        self.current_min_dist = None
        self.stop_dist = 0.64
        self.speed_const = 0.7 / 1.4

        self.movement_enabled = True
        self.current_laser_msg = None
        
        self.current_cv_image = None
        self.current_cv_rgb_image = None
        self.current_mask = None
        self.current_depth_msg = None
        self.current_rgb_image = None

        self.current_img_track = []
        self.current_depth_track = []
        self.current_target_x = None
        self.current_target_y = None
        self.current_target_depth = None

        self.crop_h = 400
        self.crop_w = 600

        self.__cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.__bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.__bumper_handler)
        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        self.__wheeldrop_sub = rospy.Subscriber('/mobile_base/events/wheel_drop',
                                                WheelDropEvent, self.__wheeldrop_handler)
        self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)
        self.__sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
        self.__led_pubs = {
            '1': rospy.Publisher('/mobile_base/commands/led1', Led),
            '2': rospy.Publisher('/mobile_base/commands/led2', Led),
        }
        self.__depth_img = rospy.Subscriber('/camera/depth/image',Image ,self.__depth_handler)
        self.__rgb_img= rospy.Subscriber('/camera/rgb/image_color',Image,self.__rgb_handler)


    def move(self, linear=0.0, angular=0.0):
        """Moves the robot at a given linear speed and angular velocity

        The speed is in meters per second and the angular velocity is in radians per second

        """
        self.__exit_if_movement_disabled()
        # Bounds checking
        if abs(linear) > self.max_linear:
            self.say("Whoa! Slowing you down to within +/-{0} m/s...".format(self.max_linear))
            linear = self.max_linear if linear > self.max_linear else linear
            linear = -self.max_linear if linear < -self.max_linear else linear
        if abs(angular) > self.max_angular:
            self.say("Whoa! Slowing you down to within +/-{0} rad/s...".format(self.max_angular))
            angular = self.max_angular if angular > self.max_angular else angular
            angular = -self.max_angular if angular < -self.max_angular else angular
        # Message generation
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        # Announce and publish
        #self.say("Moving ('{linear}' m/s, '{angular}' rad/s)...".format(linear=linear, angular=angular))
        self.__cmd_vel_pub.publish(msg)

    def move_distance(self, distance, velocity=1.0):
        """Moves a given distance in meters

        You can also give it a speed in meters per second to travel at:

            robot.move_distance(1, 0.5)  # Should take 2 seconds
        """
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            self.say("Waiting for odometry")
            r.sleep()

        msg = Twist()
        msg.linear.x = velocity
        x0 = self.__x
        y0 = self.__y
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            d = ((self.__x - x0)**2 + (self.__y - y0)**2)**0.5
            if d >= distance:
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.linear.x = 0.0
        self.__cmd_vel_pub.publish(msg)

    def turn_angle(self, angle, velocity=1.0):
        """Turns the robot a given number of degrees in radians

        You can easily convert degress into radians with the radians() function:

            robot.turn_angle(radians(45))  # Turn 45 degrees

        You can also give an angular velocity to turn at, in radians per second:

            robot.turn_angle(radians(-45), radians(45))  # Turn back over a second
        """
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            self.say("Waiting for odometry")
            r.sleep()

        msg = Twist()
        if angle >= 0:
            msg.angular.z = np.abs(velocity)
        else:
            msg.angular.z = -np.abs(velocity)
        angle0 = self.__cumulative_angle
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            a_diff = self.__cumulative_angle - angle0
            if (angle > 0 and a_diff >= angle) or (angle < 0 and a_diff <= angle):
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)

    def stop(self):
        """Stops the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.say("Stopping the robot!")
        self.__cmd_vel_pub.publish(msg)

    def wait(self, seconds):
        """This function will wait for a given number of seconds before returning"""
        self.say("Waiting for '{0}' seconds.".format(seconds))
        time.sleep(seconds)

    def say(self, msg):
        """Prints a message to the screen!"""
        print(msg)
        sys.stdout.flush()

    sounds = {
        'turn on': Sound.ON,
        'turn off': Sound.OFF,
        'recharge start': Sound.RECHARGE,
        'press button': Sound.BUTTON,
        'error sound': Sound.ERROR,
        'start cleaning': Sound.CLEANINGSTART,
        'cleaning end': Sound.CLEANINGEND,
    }

    def play_sound(self, sound_type):
        """Plays a sound on the Turtlebot

        The available sound sequences:
            - 0 'turn on'
            - 1 'turn off'
            - 2 'recharge start'
            - 3 'press button'
            - 4 'error sound'
            - 5 'start cleaning'
            - 6 'cleaning end'

        You can either pass the string or number above
        """
        if not isinstance(sound_type, (int, str)):
                self.say("!! Invalid sound type, must be an Integer or a String!")
                return
        if isinstance(sound_type, str):
            try:
                sound_type = self.sounds[sound_type]
            except KeyError:
                self.say("!! Invalid sound '{0}', must be one of: {1}"
                         .format(sound_type, self.sounds.keys()))
                return
        self.__sound_pub.publish(Sound(sound_type))

    led_colors = {
        'off': Led.BLACK,
        'black': Led.BLACK,
        'green': Led.GREEN,
        'orange': Led.ORANGE,
        'red': Led.RED,
    }

    def set_led(self, led, color):
        """Set the color of an LED

        You can set LED 1 or LED 2 to any of these colors:

        - 'off'/'black'
        - 'green'
        - 'orange'
        - 'red'

        Example:

            robot.set_led(1, 'green')
            robot.wait(1)
            robot.set_led(1, 'off')
        """
        if str(led) not in self.__led_pubs:
            self.say("!! Invalid led '{0}', must be either '1' or '2'".format(led))
            return
        if color not in self.led_colors:
            self.say("!! Invalid led color '{0}', must be one of: {1}".format(color, self.led_colors))
            return
        self.__led_pubs[str(led)].publish(Led(self.led_colors[color]))

    def reset_movement(self):
        self.movement_enabled = True

    def __odom_handler(self, msg):
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        a = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # cumulative angle doesn't wrap. assumes we've not moved more than pi radians
        # since last odom message
        if self.__have_odom:
            a_diff = a - self.__angle
            if a_diff > np.pi:
                a_diff -= 2*np.pi
            elif a_diff < -np.pi:
                a_diff += 2*np.pi
            self.__cumulative_angle += a_diff

        self.__angle = a
        self.__have_odom = True

    def __scan_handler(self, msg):
        self.current_laser_msg = msg

    def __bumper_handler(self, msg):
        if msg.state != BumperEvent.PRESSED:
            return
        if msg.bumper not in [BumperEvent.CENTER, BumperEvent.LEFT, BumperEvent.RIGHT]:
            return
        if self.on_bumper is not None:
            self.on_bumper.__call__()

    def __exit_if_movement_disabled(self):
        if not self.movement_enabled:
            self.say("Movement currently disabled")
            sys.exit()

    def __wheeldrop_handler(self, msg):
        if msg.state == WheelDropEvent.DROPPED:
            self.movement_enabled = False

    def __depth_handler(self, data):
        try:
            self.current_depth_msg = data
            self.current_cv_image = self.bridge.imgmsg_to_cv(data,"32FC1")

            h = self.crop_h / 2 
            w = self.crop_w / 2

            img = np.asarray(self.current_cv_image)

            img = img[240 - h:240 + h, 320 - w: 320 + w]
            img = img[~np.isnan(img)]

            img = np.sort(img)
            self.current_min_dist = img[0]
            
            if self.current_target_x != None and self.current_mask != None:
                img = np.asarray(self.current_cv_image)
                target_img = cv2.bitwise_and(img,img, mask = self.current_mask)
                target_img = target_img[~np.isnan(target_img)]
                target_img = np.sort(target_img)
                self.current_target_depth = target_img[ len(target_img) - 1 ]
                
            self.__check_depth()

        except CvBridgeError, e:
            print e

    def __rgb_handler(self,data):
        
        self.current_rgb_image=data
        self.current_cv_rgb_image = self.bridge.imgmsg_to_cv(data,"bgr8")
        self.__track_red()

     

    def __track_red(self):

        img = np.asarray(self.current_cv_rgb_image)
        img = cv2.blur(img,(5,5))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([170,160,60])
        upper = np.array([180,256,256])

        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.blur(mask,(5,5))
        
        self.current_mask = mask
        
        M=cv2.moments(mask)
        
        if M['m00']>0:
            if self.current_state == "searching":
                self.current_state = "following"
                song = pygame.mixer.Sound('mgs2_sound/found.wav')
                clock = pygame.time.Clock()
                song.play()

            centroid_x= int(M['m10']/M['m00'])
            centroid_y= int(M['m01']/M['m00'])
            self.current_target_x = centroid_x
            self.current_target_y = centroid_y
        
        else:

            self.current_state = "searching"

    def __check_depth(self):
        
        img = np.asarray(self.current_cv_image)
        img = img[np.isnan(img)]
        nans = len(img)

        if np.isnan(self.current_min_dist): 
            self.current_substate = "turning"
        elif self.current_min_dist < self.stop_dist:
            self.current_substate = "turning"
        elif nans > 640*480*0.5:
            self.current_substate = "backwards"
        else:
            self.current_substate = "moving"

    def move_searh_n_destroy(self, lin_velocity, angle_velocity):

        if self.current_substate == None:
            self.current_substate = "turning"
        if self.current_state == None:
            self.current_state = "following"
        
        if self.current_min_dist:
            pass#lin_velocity = self.current_min_dist*self.speed_const + self.v0
        else:
            pass#lin_velocity = 0.2

        #lin_velocity = min(self.max_linear, lin_velocity)
        if self.current_state == "searching":

            if self.current_substate == "moving":
                #self.move(linear= lin_velocity)
                pass
            elif self.current_substate == "turning":
                #self.move(angular= angle_velocity, linear=0)
                pass
            elif self.current_substate == "backwards":
                #self.move_distance(.1,-lin_velocity)
                #self.move(angular= angle_velocity)
                pass

        elif self.current_state == "following":
            if self.current_target_x != None and self.current_target_depth != None:
                factor_a = float(self.current_target_x) - 320.0
                factor_l = float(self.current_target_depth) / 2.0
                print "Vel: "+ str(lin_velocity * factor_l)
                self.move(linear = lin_velocity * factor_l, angular = angle_velocity * -1 * factor_a / 320.0 )



