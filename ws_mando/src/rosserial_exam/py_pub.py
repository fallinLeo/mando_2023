import rospy
from std_msgs.msg import String

def publish_message():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz, 매 초마다 메시지 발행

    while not rospy.is_shutdown():
        message = "Hello, Arduino!"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass