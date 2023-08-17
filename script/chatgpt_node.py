#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
import openai
import os

# Set up the proxy if you are in China
# os.environ["http_proxy"] = "http://192.168.1.13:7890"
# os.environ["https_proxy"] = "http://192.168.1.13:7890"

# Learning code: for gpt to know the programmer's code style
# Inorder to get the demanded outcome, you should modify this code accordingly
# This is a sample code to enable the turtlesim to draw a square
turtlesim_learning_code = """
#!/usr/bin/env python3

import rospy
from math import pi
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node('draw_square_node')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    vel = Twist()

    for _ in range(4):
    	# Rotate 90 degrees counterclockwise
        vel.linear.x = 0.0
        vel.angular.z = pi/2
        pub.publish(vel)
        rate.sleep()
    	
        # Move forward
        vel.linear.x = 2.0
        vel.angular.z = 0.0
        pub.publish(vel)
        rate.sleep()

    # Stop the turtle
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)"""

# This is a helper function to extract code snippets from the response
def extract_code_snippets(text):
    lines = text.strip().split("\n")
    code_snippets = []
    current_snippet = []
    in_code_block = False

    for line in lines:
        if line.startswith("```") and not in_code_block:
            in_code_block = True
        elif line.startswith("```") and in_code_block:
            in_code_block = False
            code_snippets.append('\n'.join(current_snippet))
            current_snippet = []
        elif in_code_block:
            current_snippet.append(line)

    return code_snippets


# This is the callback function of the question subscriber
def cbQuestion(msg):
    rospy.loginfo("-------------------------")
    rospy.loginfo(msg.data)

    global conversation
    # Set up the initial information to the model
    # So feel free to change this part based on your needs
    # In fact, you should conduct proper prompt engineering to make the model helpful
    global output_info, current_msg, msg_log
    output_info = [
                {"role": "system", "content": "You are a helpful assistant of a ROS1 programmer using python3. You're now helping the programmer to control turtlesim to draw a shape according to the programmer's command."},
                {"role": "system", "content": "You should only return ONE BLOCK OF python3 code snippets to the programmer. I repeat, only the code, nothing else."},
                {"role": "system", "content": "Note that since the programmer is using ROS1, you should only provide ROS1 related code snippets."},
                {"role": "system", "content": "Also the programmer is using python3, so you should only provide python3 code snippets, and you should add shibang #!/usr/bin/env python3 in the beginning of the code snippet."},
                {"role": "system", "content": "You can learn how to write code and the programmer's code style accoring to the following sample code, note that the sample code enable the turtlesim to draw a square, so you can learn to draw every other shapes with proper modifications:" + turtlesim_learning_code}
            ]
    current_msg = [{"role": "user", "content": msg.data}]
    msg_log.append({"role": "user", "content": msg.data})

    if(conversation == True):
        output_info = output_info + msg_log
    else:
        output_info = output_info + current_msg

    global model_engine
    completion = openai.ChatCompletion.create(
    # This is where you set the engine, the engine determines the model
    # You can see the models here: https://beta.openai.com/docs/engines, also listed in model.yaml
    # Refer https://platform.openai.com/docs/guides/gpt/chat-completions-api
    model = model_engine,
    # You can use msg_log to set the history, to set the personality, 
    # the temperature, top_p, frequency_penalty, presence_penalty
    # This is also how you can use the conversation mode
    messages = output_info,
    temperature = 0.2)
    response = completion["choices"][0]["message"]["content"]
    rospy.logwarn(response)
    if not response:
        return

    snippets = extract_code_snippets(response)

    # write the snippets to file, YOU MUST CHANGE THE FILE PATH TO YOUR OWN PATH!!!!!!
    file_path = "/example/example/example.py"

    # write all snippets to file
    with open(file_path, "w") as f:
        for snippet in snippets:
            f.write(snippet)
            f.write("\n")

    os.system("python3 " + file_path)
    # os.system("rm " + file_name)

    if(conversation == True):
        msg_log.append({"role": "assistant", "content": response})

    rospy.sleep(0.1)
    ready_msg = String()
    ready_msg.data = "ChatGPT is ready!"
    response_pub.publish(ready_msg)


if __name__ == "__main__":
    rospy.init_node("chatgpt_node")

    # get the parameters from launch file
    api_key =  rospy.get_param('~openai/api_key')
    model_engine =  rospy.get_param('~openai/model')
    conversation =  rospy.get_param('~openai/conversation')

    openai.api_key = api_key

    rospy.logwarn("ChatGPT: Current model %s",model_engine)

    # return the response of chatgpt: whether it returns normally or not
    global response_pub
    response_pub = rospy.Publisher("/gpt_response", String, queue_size=1)
    ready_msg = String()
    ready_msg.data = "ChatGPT is ready!"     

    # subscribe to the inquiration from user
    question_sub = rospy.Subscriber("/user_ask", String, cbQuestion, queue_size=1)

    current_msg = []
    msg_log = []
    output_info = []

    rospy.logwarn("ChatGPT: Your Name! I'm ready!")
    rospy.sleep(2)
    response_pub.publish(ready_msg)
    
    rospy.spin()