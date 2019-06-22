# Baxter MPNet-OMPL Docker

Code and container required to run OMPL-like version of MPNet motion planning algorithm in simulated environments with the Baxter Research Robot

# Running Experiments and Analyzing Data
## Build the container
In a terminal, run the following build command to build the docker image (this will take a considerable amount of time)
```
docker build -t baxter-mpnet-ompl .
```

## Run the container execution shell script to set up Baxter environment
To visualize, run ```xhost +``` in the terminal, and then launch the setup script
```
./run_image.bash
```

## Run MPNet algorithm experiment
Open a new terminal and enter the container (replace ```$CONTAINER``` with the name of the name of the container that is running the Baxter environment)
```
docker exec -it $CONTAINER bash
```

now inside of the container, source the catkin workspace that has all the resources, navigate to the experiments directory, and run the python script that sets up one of the planning scenes and calls the MPNet-OMPL planner.
```
source devel/setup.bash
roscd baxter_moveit_experiments
python motion_planning_data_gen.py $EXPERIMENT_NAME
```
replacing ```$EXPERIMENT_NAME``` with whatever name you want the data files to be saved as.

## Post-Experiment Analysis and further instruction for additional tests
The path planning data for each environment, including the paths, planning time, path cost (C-space euclidean length), and number of successful/total planning requests are recorded in a dictionary and periodically saved in the data/ folder to be analyzed or played back on the robot. ```comparison.ipynb``` in ```analysis/``` and the ```playback_path.ipynb``` notebooks are simplified examples of using the saved planning data for data analysis or visualizing the paths on the robot using the Baxter interface (ensure the robot is enabled before playing back paths, with ```rosrun baxter_tools enable_robot.py -e``` in the terminal).

To adapt MPNet within the OMPL framework, a significant amount of the algorithm's flexibility was sacrified such that experiments like that above could be easily run and reproduced to highlight the fast planning times MPNet is capable of. To run the experiment in one of the other nine environments available in the ```baxter_moveit_experiments``` repository, the local MPNet source code and experiment data generation script must be modified, and the OMPL package in the container rebuilt, such that the proper point cloud matching the test environment is loaded. 


# What's Going On?

# Docker Environment Basics
[ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter Simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

[MoveIt!](https://moveit.ros.org/install/source/)

[Baxter MoveIt! Configuration](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)

# Adapting MPNet as an OMPL planner

# Docker environment specifics (hacks necessary for everything to build)