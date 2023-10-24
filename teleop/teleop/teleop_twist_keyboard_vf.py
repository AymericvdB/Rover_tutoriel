#!/usr/bin/env python

import rclpy
#from rclpy.qos import qos_profile_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty, pty

#En lançant d'un launch, le terminal ne peut pas etre utilise pour la saisie clavier ; creation d'un autre terminal
#(pid, fd) = pty.fork()
#settings = termios.tcgetattr(fd)
settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   ,    ;    :
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   ?    .    /
t : up (+z)
b : down (-z)
anything else : stop
a/q : increase/decrease max speeds by 10%
z/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		';':(-1,0,0,0),
		':':(-1,0,0,1),
		',':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'.':(-1,0,0,0),
		'/':(-1,-1,0,0),
		'?':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'a':(1.1,1.1),
		'q':(.9,.9),
		'z':(1.1,1),
		's':(.9,1),
		'e':(1,1.1),
		'd':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	
    if args is None:
        args = sys.argv

	#rclpy.init(args)
    rclpy.init()
	
    node = rclpy.create_node('teleop_twist_keyboard')
		
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)  
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
