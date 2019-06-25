# Baxter MPNet-OMPL Docker

Code and container required to run OMPL-like version of MPNet motion planning algorithm in simulated environments with the Baxter Research Robot

# System Requirements
This container must run on a local system with a GPU and installed Nvidia drivers compatible with CUDA 9.0. It has only been successfully tested on systems with an Intel processor (systems with AMD processors may face build errors). 

# Running Experiments and Analyzing Data
## Download and unzip cuDNN files 
Download and unzip [this](https://drive.google.com/file/d/1gSWmqudfR9_tL6QGkVxBu8yuyjzt0w46/view?usp=sharing) file in the root of this repository (contains large files needed to build cuDNN inside the container)

## Build the container
In a terminal, run the following build command to build the docker image (this will take a considerable amount of time)
```
docker build -t baxter-mpnet-ompl .
```

## Run the container execution shell script to set up Baxter environment
To visualize, run ```xhost +``` in the terminal, launch the setup script, and once inside the container, source the catkin workspace and launch the Baxter MoveIt! planning environment.
```
xhost +
./run_image.bash
```
(now inside the container)
```
source devel/setup.bash
roslaunch baxter_moveit_experiments baxter_moveit.launch
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
The path planning data for each environment, including the paths, planning time, path cost (C-space euclidean length), and number of successful/total planning requests are recorded in the local ```experiment_data``` dictionary (this is mounted at a volume when the container is launched) and periodically saved in the data/ folder to be analyzed or played back on the robot. In the ```baxter_moveit_experiments``` repository (found [here](https://github.com/anthonysimeonov/baxter_moveit_experiments)), the ```comparison.ipynb``` in ```analysis/``` and the ```playback_path.ipynb``` notebooks are simplified examples of using the saved planning data for data analysis or visualizing the paths on the robot using the Baxter interface (ensure the robot is enabled before playing back paths, with ```rosrun baxter_tools enable_robot.py -e``` in the terminal).

To adapt MPNet within the OMPL framework, a significant amount of the algorithm's flexibility was sacrified such that experiments like that above could be easily run and reproduced to highlight the fast planning times MPNet is capable of. To run the experiment in one of the other nine environments available in the ```baxter_moveit_experiments``` repository, the local MPNet source code and experiment data generation script must be modified, and the OMPL package in the container rebuilt, such that the proper point cloud matching the test environment is loaded. 


# What's Going On?

# Docker Environment Basics
The docker is built combining the MoveIt! ROS package, the Baxter SDK, the OMPL motion planning library, the PyTorch machine learning library, and CUDA. The algorithm was adapted to be used within the OMPL framework such that it could access the fast  collision detection functionality provided and be relatively easily integrated with MoveIt and the Baxter robot. This requires the use of the PyTorch C++ API, as the trained neural network models used in MPNet provide the key functionality of the algorithm. Therefore, in the container the OMPL library is built with MPNet as an additional planner, and linked against Torch such that Torchscript versions of the MPNet models can be loaded in the OMPL program and used for planning. PyTorch was built from source in the container due to compiler issues linking the libraries when using the default installation of LibTorch. 

In summary, the docker builds and links the following together such that the MPNet algorithm can be executed as an OMPL planner on the Baxter with MoveIt

[ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter Simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

[MoveIt! (from source)](https://moveit.ros.org/install/source/)

[Baxter MoveIt! Configuration](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)

[PyTorch (from source)](https://pytorch.org/get-started/locally/)

[OMPL (from source)](https://github.com/anthonysimeonov/ompl)

<!-- # Adapting MPNet as an OMPL planner

# Docker environment specifics (hacks necessary for everything to build)

During the build, several local resources are copied in and such that the workspace is built properly and the experiment can run smoothly after the  -->