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

As part of the process, you will also need to install the pip package for `google-genai`. For that, you will need to run `pip3 install -q -U google-genai`. Since there is no debian package (as far as I know) for google-genai, it is perfectly fine to use `--break-system-packages` (if you get an error of an externally managed environment) for now until I figure out how to get a virtual env to work with ROS2 or until a more permanent fix is found

Now, run `colcon build --packages-select vlm_communication --symlink-install`
- `colcon build --symlink-install` works fine too, it simply would be a waste of time to build all the packages at once if we integrate with a real robot later.

Before using this project, you will need to set your API key for Gemini. Run `export GEMINI_API_KEY=<Your api key here>`. Make sure to put in your API key to this command.

If everything built successfully, congrats! Now run 
`source install/setup.bash` and  `source /opt/ros/jazzy/setup.bash` (If using Humble, run `source /opt/ros/humble/setup.bash`). Open a new terminal and run the same commands.

Now, in one terminal, run 
`ros2 run vlm_communication robot_publisher_script`

In the other terminal, run 
`ros2 run vlm_communication server_subscriber_script`


> To use with multiple computers, you will need to run this command on both PCs: `export ROS_DOMAIN_ID=<Put domain ID here>`, replacing `<Put domain ID here>` with a number. The same number has to be used on both computers. Domain IDs between the numbers 0-101 and 215-232 can be used. Right now, multi-computer support isn't great as there are bugs (like how it crashes when something doesn't correspond to what is expected due to lack of error handling), and features that are yet to be implemented (allowing for raw image bytes from a real robot, writing to files, etc.) but I plan to fix/add them in the near future.

Now, type in your prompt and put in any image paths (separated by commas) when prompted. Leave blank if no images are to be submitted. If everything was done correctly, you should be able to type your prompt in the publisher script window and an AI response should appear in the other. If that happens, great! You can try some of the demos below.

## Demos
For now, there are a couple other extra files that I have added. All of these have been tested on a WeBots R2025a simulation of the TurtleBot3, and stock TurtleBot3 navigation packages.
### Combined map image generation (no VLM)
Run the genmapimg script with `ros2 run vlm_communication genmapimg --ros-args -p map_yaml_path:=<put your map yaml path here>`, replacing `<put your map yaml path here>` with your actual map yaml path.

If all is well, it should make a map with the costmap, robot position, and navigation path overlayed on top of the global map. This map is saved to the home directory of your computer.
### Combined map image generation (with VLM)
First, run the subscriber script with `ros2 run vlm_communication server_subscriber_script`

Next, run the genmapimg script with `ros2 run vlm_communication genmapimg_with_vlm --ros-args -p map_yaml_path:=<put your map yaml path here>`, replacing `<put your map yaml path here>` with your actual map yaml path.

If all is well, it should make a map with the costmap, robot position, and navigation path overlayed on top of the global map. This map is saved to the maps folder in `<your workspace>/src/vlm_communication/vlm_communication/maps`. However, this one also queries the VLM to give a hazard rating from 1 to 10 and a description of the situation.

> <span style="color:yellow;">IMPORTANT!</span> Make sure that the .pgm file of the map you saved for navigation is in the same directory and has the same name as the YAML file, or it will not work!



That's basically it for now. More is coming in the future.
