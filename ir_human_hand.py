#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String



def main():
    LEFT_IR_TOPIC = '/robot/range/left_hand_range/state'
    RIGHT_IR_TOPIC = '/robot/range/right_hand_range/state'
    def left_range_callback(message):
        if message.range <= 0.2 and message.range > 0.12:
            pub.publish("1")
        elif message.range <= 0.12:
            pub.publish("3")

    def right_range_callback(message):
        if message.range <= 0.14 and message.range > 0.12:
            pub.publish("4")
        elif message.range <= 0.12:
            pub.publish("2")
    rospy.init_node('ir_human_node', anonymous=True)
    left_range_sub = rospy.Subscriber(LEFT_IR_TOPIC, Range, left_range_callback)
    right_range_sub = rospy.Subscriber(RIGHT_IR_TOPIC, Range, right_range_callback)
    pub = rospy.Publisher('ir_human_hand', String, queue_size=1)
    # a 10Hz publishing rate
    r = rospy.Rate(1) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        r.sleep()
    
if __name__ == '__main__':
  main()
