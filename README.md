# marmot

- mkdir -p mapf_ws/src 
- cd mapf_ws/src
- git clone git@github.com:ur10/hetero-viz.git
- catkin_make
- To visualize the markers in rviz pane, goto add button and select MarkerArray option


## Panel 1 -
 - cd mapf_ws
 - source devel/setup.bash
 - export PYTHONPATH="${PYTHONPATH}:${HOME}/mapf_ws/src/hetero-viz"
 - roslaunch marmot demonstration_driver.launch

## Panel 2 -
 - cd mapf_ws
 - source devel/setup.bash
 - export PYTHONPATH="${PYTHONPATH}:${HOME}/mapf_ws/src/hetero-viz"
 - python3 src/hetero-viz/scripts/run_robots.py

## Setting up the configs - 
- In launch/demonstration.launch set the offset parameters for positioning the agents apart on task nodes.
- The bias parameter for determinig how close the agent needs to be to the task to be considered as arrived.
- The arena scale for task distance scaling.
- ![image](https://github.com/user-attachments/assets/cfe31950-8df8-4e73-b6df-365506679b2d)




