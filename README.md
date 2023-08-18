# ROS_ChatGPT_Interaction
---
title: "ChatGPT Robot Interaction Project Report"
author: ["charles_cai@berkeley.edu","charlescaiysh@gmail.com"]
date: "2023-07"
subject: "Interaction between ROS and Large Language Model ChatGPT"
keywords: [ROS, LLM, ChatGPT, turtlesim]
subtitle: "ROS and ChatGPT"
lang: "en"
---
# Robot-ChatGPT Interaction Project Report


## Demo and Code

- Link: 
	- [CharlesCai7/ROS_ChatGPT_Interaction: This repository is for ROS ChatGPT program (github.com)](https://github.com/CharlesCai7/ROS_ChatGPT_Interaction)

https://github.com/CharlesCai7/ROS_ChatGPT_Interaction/assets/98808490/14e56f18-f913-4579-83d5-fd43e4af7e17


## Introduction

Recently, ChatGPT shows the whole world an impressive way for AI technology to be used by ordinary people. In other words, those who don't have any AI or CS knowledge can easily handle this large language model by proper prompts. This gives me inspiration that LLM can also be a efficient way for ordinary people to interact and control robots without any background about ROS. 


## Objective

The objective of this project is to connect ChatGPT with ROS, so that users can control robots by simply 'talking' to ChatGPT. This project also uses different robotic simulation platform such as turtlesim to verify if the method can be used in real world given proper prompts.


## Requirements

This project is based on certain environment:

- Ubuntu 20.04 or 18.04
- python3
- ROS1 (Since the fetch robot I use is based on ROS1 platform)
- API Key for ChatGPT


## Reference

This project refer to the following materials:

 - A Lightweight Framework for High-Quality Code  Generation
	 - https://arxiv.org/pdf/2307.08220.pdf
 - A wpr_chatgpt program
	 - [play-with-chatgpt/wpr_chatgpt: ROS package for ChatGPT (github.com)](https://github.com/play-with-chatgpt/wpr_chatgpt)
 - VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models
	 - [VoxPoser](https://voxposer.github.io/#:~:text=VoxPoser%20extracts%20affordances%20and%20constraints%20from%20large%20language,to%20zero-shot%20synthesize%20trajectories%20for%20everyday%20manipulation%20tasks.)


## Methodologies

- Use API Key to gain the access to ChatGPT
- Test and find the proper prompts to enable ChatGPT create the desired answers including codes
- Let the program create a *FILE* and write ChatGPT's code into it
- Let the program run the *FILE* automatically
- Use turtlesim to simulate a naive robot
- Experiment and polish several times


## Issues

- To get the code snippets out of ChatGPT's answer
- To make sure ChatGPT creates executable codes under specific environment
- To make sure ChatGPT creates correct and stable codes
- To let ChatGPT 'take control of' the turtlebot automatically
- To create smooth Human-Computer-Interact experience
- To do this all in one launch


## Solutions to Issues 

- Use prior system prompt to let ChatGPT create answers in specific format
- Write a function to obtain the code parts from answers
- Give ChatGPT sample codes to learn
- Give ChatGPT several specific functions to use
- Set the temperature to 0.2 for answers' stability
- Develop a conversation mode, so ChatGPT can remember the history

## Prompt Engineering

- Follow the model of: \[Context] + \[Specific information] + \[intention] + \[response format]
- Since code style varies widely using ROS, example codes is given to ChatGPT, so as to get ideal response
- Initial prompts are added as the role 'system' to give ChatGPT a personality

Here are some example prompts:
```python
{"role": "system", "content": "You are a helpful assistant of a ROS1 programmer using python3. You're now helping the programmer to control turtlesim to draw a shape according to the programmer's command."},

{"role": "system", "content": "You should only return ONE BLOCK OF python3 code snippets to the programmer. I repeat, only the code, nothing else."},

{"role": "system", "content": "Note that since the programmer is using ROS1, you should only provide ROS1 related code snippets."},

{"role": "system", "content": "Also the programmer is using python3, so you should only provide python3 code snippets, and you should add shibang #!/usr/bin/env python3 in the beginning of the code snippet."},

{"role": "system", "content": "You can learn how to write code and the programmer's code style accoring to the following sample code, note that the sample code enable the turtlesim to draw a square, so you can learn to draw every other shapes with proper modifications:" + turtlesim_learning_code}
```



## Development Potential

- Instead of turtlesim, we can apply this method to other kinds of robotics after proper alternation
- Instead of ChatGPT, a self-trained LLM specialized in robotics can improve the codes' quality and the project's suitability significantly
- Lots of optimizations can be done to improve the code generating correctness, such as given ChatGPT example functions among which it can use.
- The project can be upgraded to ROS2 platform
- Etc.






