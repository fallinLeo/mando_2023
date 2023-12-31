#!/usr/bin/env python3

from __future__ import print_function
from operator import ne
import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >


b : Emergy_Stop!! (linear_y->1)
t : auto_drive_start(linear_y->0)

anything else : stop

q/z : increase/decrease max speeds by 20%
w/x : increase/decrease only linear speed by +=1
e/c : increase/decrease only angular speed by +=50

CTRL-C to quit
"""
netural = 550
leftMax =  (30+50) + 50
rightMax = (1040-50)- 50

moveBindings = {
        'i':(1,0,0,netural),
        'o':(1,0,0,rightMax),
        'j':(0,0,0,leftMax),
        'l':(0,0,0,rightMax),#(0,0,0,-1)
        'u':(1,0,0,leftMax),
        ',':(-1,0,0,netural),
        '.':(-1,0,0,rightMax),
        'm':(-1,0,0,leftMax),
        'k':(0,0,0,netural),
        'b':(0,1,0,netural), # brake key
        't' : (0,0,0,netural)
        # 'O':(1,-1,0,0),
        # 'I':(1,0,0,0),
        # 'J':(0,1,0,0),
        # 'L':(0,-1,0,0),
        # 'U':(1,1,0,0),
        # '<':(-1,0,0,0),
        # '>':(-1,-1,0,0),
        # 'M':(-1,1,0,0),
        # 't':(0,0,1,0),
        # 'b':(0,0,-1,0),
    }

speedBindings={
        #  'q':(1,1),
        #  'z':(1,1),
        'w':(0.1,0),
        'x':(-0.1,0),
        'e':(0,50),
        'c':(0,-50),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('teleop_cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = netural
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()
        

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            twist = Twist()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.turn #self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = netural
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# Add a timer to control the message publishing rate
def publish_callback(event):
    pub_thread.update(x, y, z, th, speed, turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", netural)
    # repeat =  rospy.get_param("~repeat_rate", 10.0)
    repeat = 10
    key_timeout = rospy.get_param("~key_timeout", 0.0)

    # repeat =  rospy.get_param("~repeat_rate", 10.0)
    # key_timeout = rospy.get_param("~key_timeout", 0.5)

    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    rospy.Timer(rospy.Duration(0.1), publish_callback)  # 10 Hz


    try:
        pub_thread.wait_for_subscribers()
        # pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                turn = moveBindings[key][3] #th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]
                turn = max(150,min(850,turn))

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            # pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)