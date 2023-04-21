# ERC 2022 - Navigation Stack 

The current version of the simulation targets [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/) distribution and was mainly developed and tested on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/).
 
## Cloning of the ERC-Navigation Repository 
 
### Prerequisites 

1. [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/)
2. [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/)

The tools necessary to build this project can be installed with apt:
```
sudo apt-get install libgeographic-dev ros-noetic-geographic-msgs
sudo apt install python3-rosdep python3-catkin-tools
```
 
### Building 
 
To clone the repository:
```
git clone https://github.com/Teak-Rosewood/MRM-ERC2022-NavStack.git
```
Use the `rosdep` tool to install any missing dependencies. If you are running `rosdep` for the first time, you might have to run:
```
sudo rosdep init
```
first. Then, to install the dependencies, type:
```
rosdep update
sudo apt update
rosdep install --rosdistro noetic --from-paths src -iy
```
Now, use the `catkin` tool to build the workspace:
```sh
catkin config --extend /opt/ros/noetic
catkin build
source devel/setup.bash
```

### Running the Navigation Stack 

launching the leo rover on gazebo and rviz can be done using: 
```
roslaunch rover gazebo_rviz.launch
```
The following are the launch files to individually launch used packages: 
 
Launching robot_localization nodes:
```
roslaunch rover localization.launch
```
Launching global mapping nodes:
```
roslaunch rover mapping.launch
```
Launching ar_track_alvar node:
```
roslaunch rover ar_track_alvar.launch
```
Launching move base:
```
roslaunch rover move_base.launch 
```
As an alternative the entire navigation stack can be launched:
```
roslaunch rover navigation.launch
```
To publish a destination on the map click on the 2D nav goal button on rviz and select the destination position.

### Work to be done:
 
1. Working on a Docker Image for the Navigation Stack - 4th June (done)
2. Working on localization integration using pose from ar track data and robot_localization -4th June 
3. Working of Local planner - 4th June 
4. Working on producing a map without processing the point cloud - 4th June  
5. Working on a GUI implementation for sending Navigation Goals - 3rd Test Drive 
6. Working on sending goals in bulk with a status update and automatic probe dropping - 3rd Test Drive 
7. Setting up probe dropping - 3rd Test Drive 

## Running the Docker 
 
### 4. Pull the docker image
Once the Github workflow completes its run, you should be able to pull the image from Github container registry to your local computer for testing purposes. To do this, you will first need to [Authenticate to the Container registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry), then simply run:
```
docker pull ghcr.io/teak-rosewood/mrm-erc2022-navstack:latest
```
Replace `OWNER` with your username or organization on Github and the `IMAGE_NAME` with with the repository name. \
You can tag your commits if you want to be able to access older images. For example, if you push a `v10` tag the image will also be built with this tag and you can pull it any time by running:
```
docker pull ghcr.io/teak-rosewood/mrm-erc2022-navstack:v10
```

### 5. Run the docker image
Now, you can run and test your image:
```
docker run -it --env-file credentials.env --name mrm_img ghcr.io/teak-rosewood/mrm-erc2022-navstack
```
If everything is working correctly, the status of the device on the Freedom Robotics platform should change to `Connected`. You can then try to try to enable SSH tunnel by going into **Settings** -> **Remote SSH** and clicking **Enable Remote SSH**. Paste the ssh command into a terminal and run it, when asked for password, type `root`.

## Testing the image with the simulation
Running the image with [the simulation](https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim) is not only a means of testing your software but also a great way to create simulated scenarios of operation on the field trial.

To make your software able to communicate with the simulation nodes, you need to make sure your container and the simulation can talk on the [network](https://docs.docker.com/network/). The simplest method to do this is to just remove the container's network isolation and use the `host` network.

First, start the simulation. If you are using the simulation natively on the host you should be good to go. If you are starting the simulation through a docker container, add the `--net=host` argument to the `docker run` command.

Now, run your image, also with the `--net=host` option. The full command should look like this:
```
docker run -it --net=host --env-file credentials.env --name mrm_img ghcr.io/teak-rosewood/mrm-erc2022-navstack
```

If everything is working correctly, you should be able to visualize data from the simulation on the Freedom Robotics platform.

### Connecting to a simulation running on another computer
You can actually run this image and the simulation on different computers. This is particularly useful for testing the image on an arm-based single-board computer (like Jetson) to further replicate conditions from the actual competition.

Make sure both computers are connected to the same network and can talk to each other. You will need to start all nodes with the correct `ROS_IP` and `ROS_MASTER_URI` variables. To demonstrate it on an example, let's assume you have the following setup:
```
subnet: 10.0.0.0/24
IP of the simulation computer: 10.0.0.1
IP of the testing computer: 10.0.0.2
```
On the simulation computer, type:
```
export ROS_IP=10.0.0.1
```
If you are using simulation natively on the host, just run it. If you are running it through a docker container, add `-e ROS_IP` option to the `docker run` command to pass the variable to the container.

On the testing computer, type:
```
export ROS_IP=10.0.0.2
export ROS_MASTER_URI=http://10.0.0.1:11311
```
Add `-e ROS_IP -e ROS_MASTER_URI` arguments when running the image. The full command should look like this:
```
docker run -it --net=host -e ROS_IP -e ROS_MASTER_URI --env-file credentials.env --name mrm_img ghcr.io/teak-rosewood/mrm-erc2022-navstack
```

