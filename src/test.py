#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
class Joystick:
    def __init__(self):
        # Intializes everything
        self.lx=0
        self.az=0
        
        rospy.init_node('Joy2Turtle',annoymous=True ) # node_name
        rospy.Subscriber("joy", Joy, self.callback)
        
        self.pub = rospy.Publisher('joy_test', String, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.start()

    def callback(self,data):
        
        # vertical left stick axis = linear rate
       
        self.lx = 4*data.axes[4]
        
        # horizontal left stick axis = turn rate
        self.az = 4*data.axes[3]

    
    
    def start(self):
       
        while not rospy.is_shutdown():
            #twist=Twist()
            #twist.linear.x=self.lx
            #twist.angular.z=self.az
            if self.lx > 0.5 :
                test_str = 'forward'
            elif self.lx < -0.5 :
                test_str = 'backward'
            elif self.az > 0.5 :
                test_str = 'right'
            elif self.az < -0.5 :
                test_str = 'left'
            else :
                test_str = 'stable'
            self.pub.publish(test_str)
            print('test_str :',test_str)
            self.rate.sleep()
if __name__ == '__main__':
    try:
        Joystick()
    except rospy.ROSInterruptException:
        pass
        print('Bye')
