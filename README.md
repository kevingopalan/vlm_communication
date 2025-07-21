# vlm_communication
An interface to communicate with a VLM/LLM from ROS2

## Requirements
Ubuntu 24.04 (Might switch to 22.04)
ROS2 Jazzy (might switch to Humble, should work mostly fine already afaik)
`google-genai` pip package

## How to install
First, create a new workspace. We will call this ```vlm_ws```.

Next, make a folder called `src` inside of ```vlm_ws```.

Inside of src, clone this repository into a folder called ```vlm_communication```.

Next, go back to your workspace root (vlm_ws) and run this command in the terminal: 
```rosdep install -i --from-path src --rosdistro jazzy -y #Might switch to humble due to hardware```

Now, run ```colcon build --packages select vlm_communication --symlink-install```

If everything built successfully, congrats! Now run 
```source install/setup.bash``` (still in the root of your workspace). Open a new terminal and run the same command.
Now, in one terminal, run 
```ros2 run vlm_communication robot_publisher_script```

In the other terminal, run 
```ros2 run vlm_communication server_subscriber_script```

If everything was done correctly, you should be able to type your prompt in the publisher script window and an AI response should appear in the other. This project will turn into something bigger, and stuff will change quite a bit, including ROS versions (which is very likely). For now, here it is.
