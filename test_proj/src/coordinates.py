#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math

# a class for subscribing any message:
class msg_listener:
    def __init__(self, topic):
        rospy.Subscriber(topic, String, self.callback)
        self.rev_msg = ""

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.rev_msg = data.data

    def getdata(self):
        return self.rev_msg


class tf_listener:
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.callback)

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.rev_msg = data

    def getdata(self):
        return self.rev_msg


class laser_listener:

    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
        self.res = msg.ranges

    def getdata(self, angle):
        return self.res[int(angle)]


# coords = []
# classification = "Waste"
# ready = False
"""
 *  
 * This tutorial demonstrates simple sending of messages over the ROS system
 * and controlling a robot.
 *   
"""
# def getClass (msg):
#     global classification
#     if msg.data == "Waste" or msg.data == "Recycling":
#         classification = msg.data

# def ready (data):
#     global ready
#     if data.data > 0:
#         ready = True
#     else:
#         ready = False

def coordinates(tfmsg, dist):
    # res_pub = rospy.Publisher('/resume', String, queue_size=1)
    res_pub = rospy.Publisher('/cor_res', String, queue_size=1)
    print(tfmsg.pose.pose.orientation)
    x = tfmsg.pose.pose.position.x
    y = tfmsg.pose.pose.position.y

    orientation = tfmsg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    print("robot: (x=%.3f, y=%.3f, theta=%.3f, dist=%.3f)" % (x, y, theta,dist))

    dist += 0.05
    angle = theta*180/math.pi
    if angle >= 0 and angle < 90:
        xcor = x+ (dist * math.cos(theta))
        ycor = y+ (dist * math.sin(theta))
    elif angle >= 90 and angle <= 180:
        theta = (180-angle)*math.pi/180
        xcor = x- (dist * math.cos(theta))
        ycor = y+ (dist * math.sin(theta))
    elif angle < 0 and angle >= -90:
        theta = (-1*angle)*math.pi/180
        xcor = x+ (dist * math.cos(theta))
        ycor = y- (dist * math.sin(theta))
    else:
        theta = (180+angle)*math.pi/180
        xcor = x- (dist * math.cos(theta))
        ycor = y- (dist * math.sin(theta))

    cor_msg = "%.3f,%.3f" % (xcor, ycor)
    res_pub.publish(cor_msg)
    rospy.loginfo(cor_msg)

def coordinates_dummy():
    res_pub = rospy.Publisher('/cor_res', String, queue_size=1)
    
    xcor = 0
    ycor = 0

    cor_msg = "%.3f,%.3f" % (xcor, ycor)
    res_pub.publish(cor_msg)
    rospy.loginfo(cor_msg)
    

if __name__ == "__main__":
    # Name your node
    rospy.init_node("coordinates", anonymous=True)

    # Publisher object that decides what kind of topic to publish and how fast.
    # classification_pub = rospy.Publisher("/classification", String, queue_size=1)

    # rospy.Subscriber('/tf_ready', Int8, ready)
    # rospy.Subscriber('/odom', Odometry, coordinates)
    # rospy.Subscriber("/classification", String, getClass)
    ready_listener = msg_listener("/calcord")
    tf_listener = tf_listener()
    l_listener = laser_listener()
    tf = rospy.Publisher("/cor_res", String, queue_size=1)
    cal = rospy.Publisher("/calcord", String, queue_size=1)

    rate = rospy.Rate(1)
    rate.sleep()
    tf.publish("")

    # rospy.spin()
    while not rospy.is_shutdown():
    	if ready_listener.getdata() == "ready":
    		print("do calculation")
    		coordinates(tf_listener.getdata(), l_listener.getdata(0))
    		# tf.publish("wait")
    		cal.publish("done")
    	rate.sleep()