# vlm_communication
An interface to communicate with a VLM/LLM (gemini-2.5-flash) from ROS2, written in Python.

## Requirements
Ubuntu 24.04/22.04

ROS2 Jazzy/Humble

## How to use
First, create a new workspace. We will call this `vlm_ws`.

Next, make a folder called `src` inside of `vlm_ws`.

Inside of src, clone this repository into a folder called `vlm_communication`.

Now, source your ROS2 installation with `source /opt/ros/jazzy/setup.bash` (or if on Humble, run `source /opt/ros/humble/setup.bash`)

Next, go back to your workspace root (vlm_ws) and run this command in the terminal: 
`rosdep install -i --from-path src --rosdistro jazzy -y` (or if on Humble, run `rosdep install -i --from-path src --rosdistro humble -y`)

As part of the process, you will also need to install the pip package for `google-genai`. For that, you will need to run `pip3 install -q -U google-genai`. Since there is no debian package (as far as I know) for google-genai, it is perfectly fine to use --break-system-packages (if you get an error of an externally managed environment) for now until I figure out how to get a virtual env to work with ROS2 or until a more permanent fix is found

Now, run `colcon build --packages-select vlm_communication --symlink-install`
- `colcon build --symlink-install` works fine too, it simply would be a waste of time to build all the packages at once if we integrate with a real robot later.

Before using this project, you will need to set your API key for Gemini. Run `export GEMINI_API_KEY=<Your api key here>`. Make sure to put in your API key to this command.

If everything built successfully, congrats! Now run 
`source install/setup.bash` and  `source /opt/ros/jazzy/setup.bash` (If using Humble, run `source /opt/ros/humble/setup.bash`). Open a new terminal and run the same commands.
Now, in one terminal, run 
`ros2 run vlm_communication robot_publisher_script`

In the other terminal, run 
`ros2 run vlm_communication server_subscriber_script`

Now, type in your prompt and put in any image paths (separated by commas) when prompted. Leave blank if no images are to be submitted. If everything was done correctly, you should be able to type your prompt in the publisher script window and an AI response should appear in the other. This project will turn into something bigger, and stuff will change quite a bit, including ROS versions (which is very likely). For now, here it is.
