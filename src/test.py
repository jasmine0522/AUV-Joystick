#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy

'''
rosrun joy joy_node
'''

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
        self.K = 4      # to enlarge the result
        self.ly = 0
        self.rx = 0
        self.ry = 0
        
        rospy.init_node('Joy2Turtle',anonymous=True ) # node_name
        rospy.Subscriber("joy", Joy, self.callback)
        
        self.pub = rospy.Publisher('cmd', String, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.start()
        

    def callback(self,data):             
        self.ly = data.axes[5] * self.K     # {0, -1, 1}
        self.rx = data.axes[2] * self.K     # [-1, 1]
        self.ry = data.axes[3] * self.K     # [-1, 1]
        # print(data)
    
    
    def start(self):       
        while not rospy.is_shutdown():
            if self.ry > 0.5:
                cmd_str = 'forward'
            elif self.ry < -0.5:
                cmd_str = 'backward'
            elif self.rx > 0.5:
                cmd_str = 'left'
            elif self.rx < -0.5:
                cmd_str = 'right'
            elif self.ly > 0.5:
                cmd_str = 'up'
            elif self.ly < -0.5:
                cmd_str = 'down'
            else:
                cmd_str = 'stop'
            self.pub.publish(cmd_str)
            print('cmd_str :',cmd_str)
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Joystick()
    except rospy.ROSInterruptException:
        print('Bye')
