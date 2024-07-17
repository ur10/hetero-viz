# marmot

- mkdir -p mapf_ws/src 
- cd mapf_ws/src
- git clone git@github.com:ur10/hetero-viz.git
- catkin_make


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


