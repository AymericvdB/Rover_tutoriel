#!/usr/bin/env python

import rclpy
#from rclpy.qos import qos_profile_default
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

msg_empty = """
Reading from the keyboard  and Publishing to Twist! Adapte pour Gazebo
---------------------------
Moving around:
   u    i    o
   j    k    l
   ,    ;    :
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   r    t    y
   f    g    h
   v    b    n
p : up (+z)
m : down (-z)
anything else : stop
a/q : increase/decrease max speeds by 10%
z/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
		73:(1,0,0,0),#i
		79:(1,0,0,-1),#o
		74:(0,0,0,1),#j
		76:(0,0,0,-1),#l
		85:(1,0,0,1),#u
		59:(-1,0,0,0),#;
		58:(-1,0,0,1),#:
		44:(-1,0,0,-1),#,
		89:(1,-1,0,0),#y
		84:(1,0,0,0),#t
		70:(0,1,0,0),#f
		72:(0,-1,0,0),#h
	    82:(1,1,0,0),#r
		66:(-1,0,0,0),#b
		78:(-1,-1,0,0),#n
		86:(-1,1,0,0),#v
		80:(0,0,1,0),#p
		66:(0,0,-1,0),#m
	       }

speedBindings={
		65:(1.1,1.1),#a
		81:(.9,.9),#q
		90:(1.1,1),#z
		83:(.9,1),#s
		69:(1,1.1),#e
		68:(1,.9),#d
	      }

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)


class Teleop_Gazebo(Node):

    def __init__(self):

        self.speed = 1.0
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

        self.key = 0

        super().__init__('teleop_twist_gazebo')

        self.pub_command_twist = self.create_publisher(Twist, '/rover_base2/cmd_vel', 10)
        self.pub_command = self.create_publisher(Odometry, '/rover_base2/command/pos_vel', 10)
        self.pub_command_mode = self.create_publisher(Int32, "/rover_base2/command/mode", 10)

        self.sub_keyboard = self.create_subscription(Int32, '/keyboard/keypress', self.enregistrement, 10)
        self.sub_keyboard  # prevent unused variable warning

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.listener_callback)

    def enregistrement(self, msg):

        self.key = msg.data

    def listener_callback(self):
        #self.get_logger().info('I heard: "%s"' % msg.data)

        try:
            #self.key = msg.data
            if self.key in moveBindings.keys():
                self.x = moveBindings[self.key][0]
                self.y = moveBindings[self.key][1]
                self.z = moveBindings[self.key][2]
                self.th = moveBindings[self.key][3]
            elif self.key in speedBindings.keys():
                self.speed = self.speed * speedBindings[self.key][0]
                self.turn = self.turn * speedBindings[self.key][1]
            
                print(vels(self.speed,self.turn))
                if (self.status == 14):
                    print(msg_empty)  
                self.status = (self.status + 1) % 15
            else:
                self.x = 0
                self.y = 0
                self.z = 0
                self.th = 0

            cmd_odom = Odometry()
            cmd_mode = Int32()
            cmd_twist = Twist()

            cmd_mode.data = 1
            cmd_odom.twist.twist.linear.x = self.x*self.speed; cmd_odom.twist.twist.linear.y = self.y*self.speed; cmd_odom.twist.twist.linear.z = self.z*self.speed
            cmd_odom.twist.twist.angular.x = 0.0; cmd_odom.twist.twist.angular.y = 0.0; cmd_odom.twist.twist.angular.z = self.th*self.turn
            
            cmd_twist.linear.x = cmd_odom.twist.twist.linear.x; cmd_twist.linear.y = cmd_odom.twist.twist.linear.y; cmd_twist.linear.z = cmd_odom.twist.twist.linear.z
            cmd_twist.angular.x = cmd_odom.twist.twist.angular.x; cmd_twist.angular.y = cmd_odom.twist.twist.angular.y; cmd_twist.angular.z = cmd_odom.twist.twist.angular.z
            
            self.pub_command_mode.publish(cmd_mode)
            self.pub_command.publish(cmd_odom)
            self.pub_command_twist.publish(cmd_twist)

        except:
            print("erreur")

            cmd_odom = Odometry()
            cmd_mode = Int32()

            cmd_mode.data = 1
            cmd_odom.twist.twist.linear.x = 0.0; cmd_odom.twist.twist.linear.y = 0.0; cmd_odom.twist.twist.linear.z = 0.0
            cmd_odom.twist.twist.angular.x = 0.0; cmd_odom.twist.twist.angular.y = 0.0; cmd_odom.twist.twist.angular.z = 0.0
            
            cmd_twist.linear.x = cmd_odom.twist.twist.linear.x; cmd_twist.linear.y = cmd_odom.twist.twist.linear.y; cmd_twist.linear.z = cmd_odom.twist.twist.linear.z
            cmd_twist.angular.x = cmd_odom.twist.twist.angular.x; cmd_twist.angular.y = cmd_odom.twist.twist.angular.y; cmd_twist.angular.z = cmd_odom.twist.twist.angular.z

            self.pub_command_mode.publish(cmd_mode)
            self.pub_command.publish(cmd_odom)
            self.pub_command_twist.publish(cmd_twist)

        #finally:
        #    cmd_odom = Odometry()
        #    cmd_mode = Int32()

        #    cmd_mode.data = 1
        #    cmd_odom.twist.twist.linear.x = 0.0; cmd_odom.twist.twist.linear.y = 0.0; cmd_odom.twist.twist.linear.z = 0.0
        #    cmd_odom.twist.twist.angular.x = 0.0; cmd_odom.twist.twist.angular.y = 0.0; cmd_odom.twist.twist.angular.z = 0.0

        #    self.pub_command_mode.publish(cmd_mode)
        #    self.pub_command.publish(cmd_odom)

def main(args=None):	
	
    rclpy.init()

    teleop_gazebo = Teleop_Gazebo()

    rclpy.spin(teleop_gazebo)

    teleop_gazebo.destroy_node()
    rclpy.shutdown()
	
if __name__ == '__main__':
    main()