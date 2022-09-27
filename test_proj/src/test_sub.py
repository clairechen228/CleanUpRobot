#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import re
import random

# a class for subscribing any message:
class listener:
    def __init__(self, topic):
        rospy.Subscriber(topic, String, self.callback)
        self.rev_msg = ""

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.rev_msg = data.data

    def getdata(self):
        return self.rev_msg

    
def main():

    rospy.init_node('listener', anonymous=True)

    rec_colors = []

    rate = rospy.Rate(1)

    cv_listener = listener("rec_result") # listen to cv algirthm result, format: shaple,color,x,y;
    tf_listener = listener("tf_ready") # listen to auto walk node when rotation is done
    cor_listener = listener("cor_res") # listen to transform algrithm that gives the coordinates of the garbage
    corready_listener = listener("calcord") # listen to transform algrithm that gives the coordinates of the garbage

    stop_autowalk_pub = rospy.Publisher("resume", String, queue_size=10) # publish to auto walk to stop it
    calcord_pub = rospy.Publisher("calcord", String, queue_size=10) # publish to coordinate node to start calculating

    rate.sleep()

    while not rospy.is_shutdown():
        # first read the cv result:
        cv_res = cv_listener.getdata()
        if cv_res != "":
            objs = cv_res.split(";")
            for obj in objs:
                if "," not in obj:
                    continue
                splited = obj.split(",")
                shape = splited[0]
                color = splited[1]
                area = float(splited[2])
                if area < 0.03 or area > 0.5: # too small or too large may influence the cv
                    continue
                x = float(splited[3])
                y = float(splited[4])
                print("Detected one obj: shape = %s, color = %s, center = [%.2f,%.2f]" % (shape,color,x,y))
                if color in rec_colors:
                    print("Color already reced, move on")
                    continue

                obj_type = ""
                if shape=="circle":
                    obj_type = "Recycling"
                elif shape=="rectangle":
                    obj_type = "Waste"
                else:
                    continue

                if obj_type != "":
                    print("stop auto walk, calculate coordinates:")
                    # pub to walk node:
                    stop_autowalk_pub.publish("pause")
                    # wait for rotate until towards garbage:
                    while True:
                        if not tf_listener.getdata() == "ready":
                            rate.sleep()
                        else:
                            break
                    # wait for transformation node's coordinates:
                    calcord_pub.publish("ready")
                    while True:
                        # wait until coordinate saids done
                        while not corready_listener.getdata() == "done":
                            rate.sleep()
                        print("calculation done")
                        tf_res = cor_listener.getdata()
                        print(tf_res)
                        if "," in tf_res:
                            calcord_pub.publish("wait")
                            splited = tf_res.split(",")
                            xcor = float(splited[0])
                            ycor = float(splited[1])
                            f = open ("/mnt/c/Users/garba/Documents/Robotic/project.txt", "a+")
                            f.write(obj_type + "[" + color + "]: ("+ str(xcor) + "," + str(ycor) + ");\n")
                            f.close()
                            break
                        rate.sleep()
                    stop_autowalk_pub.publish("done")
                    rec_colors.append(color)
        rate.sleep()

if __name__ == '__main__':
    main()