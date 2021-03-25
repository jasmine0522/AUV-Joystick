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
def callback(data):
    twist = Twist()
    # vertical left stick axis = linear rate
    print('axis[6]=',data.axes[6])
    print('axis[7]=',data.axes[7])
    twist.linear.x = 2*data.axes[7]
    
    # horizontal left stick axis = turn rate
    twist.angular.z = 2*data.axes[6]

    pub.publish(twist)
    rate.sleep()
# Intializes everything
def start():
    
    # starts the node
    rospy.init_node('Joy2Turtle')
   
    # define rate(hz)
    global rate
    rate = rospy.Rate(10)
    
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist,1)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    
    rospy.spin()

if __name__ == '__main__':
    start()
    
