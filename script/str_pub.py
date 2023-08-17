#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String


def callback(msg):
    rospy.loginfo("Enter your inquiry:(q to quit)")
    if msg.data != "ChatGPT is ready!":
        rospy.loginfo("Something went wrong!")
        return
    
    while not rospy.is_shutdown():
        inquiry = input("User: ")
        if(len(inquiry) > 1):
            question_msg = String()
            question_msg.data = inquiry

            # Publish the question to the chatgpt_node
            ask_pub = rospy.Publisher("/user_ask", String, queue_size=1)
            rospy.sleep(0.1)
            ask_pub.publish(question_msg)
            break
        elif (len(inquiry) == 1 and inquiry == 'q'):
            rospy.loginfo("Bye!")
            break
        else:
            rospy.logwarn("Please input your question!")


if __name__ == "__main__":
    rospy.init_node("str_pub")

    ans_sub = rospy.Subscriber("/gpt_response", String, callback, queue_size=1)

    rospy.spin()
