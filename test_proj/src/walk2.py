#!/usr/bin/env python3
""" 
 * CSCI5551 - Project - Autowalk
 * Claire Chen <chen6242>
 * May **, 2021
"""
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

class Robot:
    def __init__(self):
        self.pos_x = None   #
        self.pos_y = None   #
        self.angle = None   #
        self.dir = None     #
        self.laser = None
        self.cloest_dis = None
        self.cloest_angle = None
        self.check = None
        self.speed = 0.2
        self.dest_x = None
        self.dest_y = None
        self.theta = None
        self.avoid = 0.3 #how far away from obstacles
        self.mode = 'mode1'
        self.scan = None
        self.c = 0
        #11 points
        self.path = [(-1,3.5,90),(-6,4,270),(-6,-3.3,90),(-6,4,250),(-4,1,180),(3,0.5,120),(1,4.5,270),(1,2,90),(3,4.5,200),(6.5,0.5,45),(5.4,-4.5,90)]
        self.track = 0
        self.pause = False
        #subsrciber/publisher
        self.base_pos = None
        self.cmd_vel_pub = None
        self.twist = None
        self.CV = None
        self.tf = None
        self.com = None 
        self.resume = None

    def start(self):
        self.set_goal()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.base_pos = rospy.Subscriber('/odom', Odometry, self._callback)
        self.scan = rospy.Subscriber('/scan', LaserScan, self._callback2)
        self.CV = rospy.Subscriber('rec_result', String, self.callback3)
        self.tf = rospy.Publisher("/tf_ready", String, queue_size=1)
        self.com = String()
        self.resume = rospy.Subscriber('/resume', String, self.callback4)

        while not rospy.is_shutdown():  # running until being interrupted manually
            continue

    def set_goal(self):
        goal = self.path[self.track]
        self.dest_x = goal[0]
        self.dest_y = goal[1]
        self.theta = goal[2]

    def _callback(self, msg):  #update everytime
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.angle = 180-(yaw*180/math.pi)
        self.calc_direction()
        # print('callback odom: ', self.pos_x, self.pos_y, self.angle, self.dir)

    def calc_direction(self):  #update everytime
        Y = self.dest_y-self.pos_y
        X = self.dest_x-self.pos_x
        dir = math.atan(Y/X)*180/math.pi
        if X >= 0 and Y >= 0:
            self.dir = 180-dir
        elif X < 0 and Y >= 0:
            self.dir = -1*dir
        elif X < 0 and Y < 0:
            self.dir = 360-dir
        elif X >= 0 and Y < 0:
            self.dir = 180-dir
        else:
            raise Exception("CANT CALCULATE THE DIRECTION")

    def go(self, speed):
        self.twist.linear.x = speed
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def turn (self,angular_speed):
        self.twist.linear.x = 0
        self.twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.twist)
   

    def _callback2(self, msg):
        #If the CV algorithm detects a trash
        if self.pause:
            pass
        else:
            inf = math.inf
            laser = msg.ranges
            self.laser = laser
            #Detect the distance and the orientation of the cloest obstacle
            cloest_angle_left = np.argmin(laser[:95])
            cloest_angle_right = np.argmin(laser[265:])+265
            if laser[cloest_angle_left] <= laser[cloest_angle_right]:
                cloest_angle = cloest_angle_left
            else:
                cloest_angle = cloest_angle_right
            self.cloest_dis = laser[cloest_angle]
            self.cloest_angle = cloest_angle

            #Check the distance at the orientation of the goal 
            if self.angle >= self.dir:
                self.check = round(abs(self.angle-self.dir))
            elif self.angle < self.dir:
                self.check = round(abs(360+self.angle-self.dir))
            else:
                raise Exception(('unknown. check angle'))
            if self.check == 360:
                self.check = 0
            
            # print('callback laser: cls_dis, cls_ang, check = ', self.cloest_dis, self.cloest_angle, self.check)
            if self.reach():
                self.stop()
                while not self.final_pose():
                    pass
                self.track += 1
                if self.track == len(self.path):
                    print('DONE!')
                else:
                    self.set_goal()
            else:
                self.decide()

    
    def decide(self):
        if self.mode == 'mode1' and min(min(self.laser[:91]), min(self.laser[270:])) <= self.avoid:
            self.stop()
            a = np.argmin(self.laser[:45])
            b = np.argmin(self.laser[315:360])
            if self.laser[a] >= self.laser[315+b]:
                c = 315+b
                t = 'left'
            else:
                c = a
                t = 'right'
            self.mode = 'mode2'
            self.mode2(c,t)
        elif self.mode == 'mode2' and ((round(self.cloest_angle) >= 88 and round(self.cloest_angle) <= 92) or (round(self.cloest_angle) >= 268 and round(self.cloest_angle) <= 272)):
            self.go(0.15)
            self.mode = 'mode3'
            self.c = 0
        elif self.mode == 'mode2':
            pass
        elif self.mode == 'mode3' and self.c <7:
            self.c += 1
        elif self.mode == 'mode3' and self.laser[self.check] > self.avoid+0.1:
            self.stop()
            self.mode = 'mode1'
            self.mode1()
        elif self.mode == 'mode3':
            pass
        elif self.mode == 'mode1':
            self.mode1()
        else:
            raise Exception('why')
        
        # print('mode, speed: ', self.mode, self.twist.linear.x, self.twist.angular.z)

    def mode1(self):
        #face the goal -> go
        if round(self.dir-self.angle) >= -1 and round(self.dir-self.angle) <= 1:
            self.go(self.speed)
        elif round(self.dir-self.angle) <= 180:
            #turn right
            self.turn(-1*(self.dir-self.angle)/180*math.pi)
        elif round(360+self.dir-self.angle) <= 180:
            #turn right
            self.turn(-1*(360+self.dir-self.angle)/180*math.pi)
        elif round(self.angle-self.dir) < 180: 
            #turn left
            self.turn((self.angle-self.dir)/180*math.pi)
        elif round(360+self.angle-self.dir) < 180:
            #turn left
            self.turn((360+self.angle-self.dir)/180*math.pi)
        else:
            raise Exception('Not sure turn left or right')

    def mode2(self,c,t):
        # self.turn(-1*abs(math.cos(self.cloest_angle*math.pi/180)*0.3+0.07))
        if t == 'left':
            self.turn(0.1)
        else:
            self.turn(-0.1)
 
    def reach(self):
        if round(self.pos_x,1) == self.dest_x and round(self.pos_y,1) == self.dest_y:
            return True
        else:
            return False

    def final_pose(self):
        diff = self.theta-self.angle
        self.twist.linear.x = 0
        self.twist.angular.z = -1*diff*math.pi/180
        self.cmd_vel_pub.publish(self.twist)
        if abs(diff) < 0.5:
            self.stop()
            return True
            # print('You reach the goal at (x,y,theta) = ', self.pos_x, self.pos_y, self.theta)
        else:
            False

    def callback3(self,data):
        if self.pause:
            vals = data.data.split(",")
            x = float(vals[3])
            print(x)
            if round(x,1) == 0.0:   #tolerance
                self.stop()
                self.com = "ready"
                self.tf.publish(self.com)
                rospy.loginfo("ready to calculate")
            elif x > 0:
                #target object is on the right side -> rotate clockwise
                self.turn(-0.05)
            elif x < 0:
                #target object is on the left side -> rotate counter-clockwise
                self.turn(0.05)

    def callback4(self,data):
        ans = data.data
        if ans == 'done':
            print("Resume walking")
            self.com = "wait"
            self.tf.publish(self.com)
            self.pause = False
            #resume autowalk
        elif ans == "pause":
            print("Pausing")
            self.pause = True
        else:
            pass
    




    
def main():
    
    rospy.init_node("bug0", anonymous=True)  
    robot = Robot()
    robot.start()      


if __name__ == "__main__":
    main()
