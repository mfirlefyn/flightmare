# Flightmare: Direct Learning of Home Vector Direction for Insect-inspired Robot Navigation 

![alt text](https://github.com/mfirlefyn/flightmare/blob/master/docs/scene_git.png "intro image")

## Clean Installation Instructions

Taken from the original [Flightmare](https://flightmare.readthedocs.io/en/latest/) documentation.

This guide is meant for bearingnet (approach from paper) and Flightmare installation on a fresh Ubuntu 20.04 machine. It is not excluded that the installation procedure will work on a not-so-fresh machine. It may also work on other Ubuntu or Linux flavors. This is left for the reader. 

Just follow along with the guide and copy paste the lines in the terminal after the previous command has finished. You will not have to open other terminal sessions unless explicitly stated. Whenever lines have to be pasted line-by-line in terminal, the instruction description will have a "(!)" indication. If the instruction lines can be copy pasted in the terminal as a whole, no further distinction is made.

Most of the installation instructions remain unchanged with respect to the original. It must be noted that there were some discrepencies as we installed Flightmare for the first time, which will be noted along the way. Let's open up a terminal and get started.

### Flightmare Prerequisites

#### Git

Make sure your machine has up-to-date resources:
```console
sudo apt update
```

When this instruction returned "All packages are up to date." you may skip the following instruction. Otherwise, run the following to upgrade your packages:
```console
sudo apt upgrade -y
```

All packages are up-to-date, now we can start installing git. As good practice your name and email may be configured by substituting in the appropriate fields (!):
```console
sudo apt install git -y
git config --global user.name "Your Name Here"
git config --global user.email "Same Email as used for Github Here"
git config --global color.ui true
```

#### Dependencies

Flightmare depends on CMake and the GCC compiler. Also some Python3 packages need to be installed, including OpenMPI and OpenCV. Let's run:
```console
sudo apt install -y --no-install-recommends \
	build-essential \
	cmake \
	libzmqpp-dev \
	libopencv-dev
```

### Flightmare Installation with ROS

Some of the readers may wonder why we stray from the original instructions here. In the original, there are 2 ways in which can install the program: via Python or via ROS. The Python install is very convenient for ML applications since it works fast and without much issues. However, Python depends on a lot of C/C++ wrappers and you may find that some of the wrappers still need to be written. Therefore, if you want to have control over your application it is highly advised to focus on C++ development.

#### ROS
ROS comes in many different versions depending on your OS. So, make sure you pick the flavor that fits your OS. As we are using Ubuntu 20.04 for this guide, we focus on the installation of ROS Noetic Ninjemys.

The following listing of instructions is taken literally from the ROS website.

Make sure to configure your Ubuntu repositories to allow "restricted", "universe", and "multiverse" packages. Follow the Ubuntu [website link](https://help.ubuntu.com/community/Repositories/Ubuntu) for the appropriate instructions on how to do this. You can use the link just to check whether this is the case. Normally they are all already toggled on by default.

Add the ROS Ubuntu repository to your sources.list:
```console
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Install curl and add the necessary ROS keys (!):
```console
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

The terminal should now display "OK". That means we can continue to the installation. Just update the system's resources again since we have added ROS:
```console
sudo apt update
```

You can see the added ROS packages in the listing. Now we execute the command to start the full desktop install of ROS Noetic. It is the recommended install. Check the ROS website if you prefer other ROS package setups during installation. For now, run:
```console
sudo apt install ros-noetic-desktop-full -y
```

Now, in order to use ROS in a terminal session we need to source the following script. This needs to be done every time you open a new terminal session:
```console
source /opt/ros/noetic/setup.bash
```

To create and manage your own ROS workspaces, we need some additional dependencies:
```console
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

We are not using the rosdep package explicitly during this guide. Just for kicks let us check whether the ROS install actually works with rosdep (!):
```console
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

All checked out? No crazy, unexpected terminal outputs? Alright, let's continue.

#### ROS Dependencies for Flightmare

For those attentive readers among you that are following along with the original instructions and these instructions side-by-side, you may notice that we are diverting from the original starting here. This has several reasons, chief among them being that the original did not work for us. Probably due to the usage of different ROS packages.

Alterations are only small. For now, we just skip the Gazebo installation. If it is not installed with ROS by default, we will install it whenever the terminal complains about it not being present.

Install the ROS and system dependencies:
```console
sudo apt install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool -y
```

If you are following along with the original, stop it already! Just kidding. They would have you change the protobuf compiler version to 3.0.0. Do not attempt this. It will just mess up your installation for this setup. You can check your version with:
```console
protoc --version
```

It should display "libprotoc 3.6.1" and that is just fine.

#### Catkin Tools for ROS Workspace

We first install pip package manager for Python3 and then install the Catkin workspace tools (!): 
```console
sudo apt install python3-pip -y
sudo pip install catkin-tools
```

#### ROS Workspace

ROS workspace uses Catkin workspace tools as an intermediary. To create the workspace and configure it, just enter (!):
```console
cd
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The terminal should display that the configuration is valid and that the catkin workspace has been initialized. All good so far. Nearly there.

#### Flightmare

Now, the repository can be cloned in the workspace. You can use the original repository or in our case we want to include all the appropriate changes for our simulation (!):
```console
cd ~/catkin_ws/src
git clone https://github.com/mfirlefyn/flightmare
```

Clone the dependencies into the workspace:
```console
vcs-import < flightmare/flightros/dependencies.yaml
```

At this point, especially if you are using the original instructions, it is possible that git returns something like: "The authenticity of the host can't be established. ... Are you sure you want to continue?" In this case, just type "no" and press enter. The terminal returns a bunch of "fatal: Could not read from remote respository" errors. Be aware that the original "dependencies.yaml" file uses some ssh addresses for the Github repos in the "url" fields. Replacing everything in the url fields with the actual package https url from Github should resolve these issues. This should actually be ready and done for you if you cloned our repository.

The last step of the Flightmare installation procedure is to build the workspace:
```console
catkin build
```

The build process should complete without any errors. You may encounter some warnings in the process. It is fine to ignore them for now. You only need to continue with these last instructions if you had any issues during the build process and want to build again. 

The workspace needs to be rebuild if you made changes to the code within it or if something went horribly wrong and you need to start over in the same folder. It is good practice to clean the workspace before the build command is run again, such that no lingering faults remain in the workspace (!):

```console
catkin clean
catkin build
```

If you encounter any errors that relate to certain ROS packages that were not present in your current installation, for example Gazebo is missing, you can try to install the package manually with ```sudo apt install ros-noetic-gazebo-ros```. It is also possible to check out a listing of all available ros packages by running ```apt search ros-noetic```.

#### Flightmare Unity Renderer Binary

Flightmare is installed, but that does not mean that it can be used immediately. The Unity Renderer needs to be in the appropriate directory to start up the simulation. Thus, first download the Unity renderer binary from our [Releases](https://github.com/mfirlefyn/flightmare/releases) page (The name will be "RPG_Flightmare.tar.xz"). Assuming that the tar library is now in your ~/Downloads directory, we can simply run the following to extract it to the right destination (!):
```console
cd ~/catkin_ws/src/flightmare/flightrender
tar -xvf ~/Downloads/RPG_flightmare.tar.xz
```

Congratulations! You have successfully installed Flightmare and are ready to proceed to the Python environment installation.

#### Testing Flightmare

As a little break from installing stuff, you can try to start up Flightmare and confirm it actually works like expected.

Navigate to the Unity renderer directory:
```console
nautilus ~/catkin_ws/src/flightmare/flightrender/RPG_Flightmare
```

When the file explorer opens in the required directory, double click on "RPG_Flightmare.x86_64". Give it some time to start up. If nothing happens, try closing the window and consequently opening it again.

It may happen that the Unity renderer window just freezes without you being able to close it. This may happen to any consequent windows that you try to open, since the program is not what you would consider stable software. It will work eventually. A workaround that we tend to use quite often is to check the the program ID (PID) in an htop process overview window, note it, and kill it manually (!):
```console
sudo apt update
sudo apt install htop -y
htop
```

Use "Ctrl+C" to close the interactive console window and return to normal console. Note the PID of where it gives you the Unity renderer path (ending on "RPG_Flightmare.x86_64")  in the command field and insert it after the kill instruction:
```console
kill -9 PID
``` 

Where "PID" in the instruction is replaced by the actual PID number, 17929 for example:
```console
kill -9 17929
``` 

If you succeeded in opening the renderer, click "Forest" in the menu to enter the Forest environment we developed. Be patient for the renderer to change the displayed environment. Once changed, click "Scene Save Pointcloud". Now you can navigate the simulated world with the "awsd" keys. The "qe" keys control directional rotation. The "tg" keys control longitudinal rotation. Lastly, the "rf" keys can be used to go up and down. The "h" key can be used to toggle visibility of the cube that represents the nest. 

Your window should look similar to the following: ![alt text](https://github.com/mfirlefyn/flightmare/blob/master/docs/screenshot_forestenv.png "Screenshot Forest Env")

To run the Flightmare simulation, open a terminal and enter the following (!):
```console
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch flightros camera.launch
```

Like with running the standalone renderer, it may happen that the renderer hangs on opening. The terminal will print a summary of the initialization process and will then indicate "[UnityBrudge] Trying to Connect Unity. [............" and so on. If the renderer does not respond within 30-60 seconds, you can assume that the renderer hangs again and close it down using the kill instruction. If you scroll up in the printed summary, you will have 2 lines indicating the PID of the renderer process and the camera simulation program running in Flightmare. This is "process[rpg_flightmare_render-2]: started with pid [3145]" and "process[camera-3]: started with pid [3150]" respectively. Use your PID numbers instead and run the kill commands in a separate terminal (!):
```console
kill -9 3150
kill -9 3145
```

If these processes are killed, then you should have the first terminal available again. Retry the roslaunch command in the first terminal:
```console
roslaunch flightros camera.launch
```

Iterate the kill and start up process until the renderer starts up properly. The renderer needs to start up in order to tell the camera program that it is ready to receive the next instructions. Otherwise the camera program that you are trying to run in your first terminal just continues printing "....." until the timeout and it will kill the running processes by itself. Although, this may take a lot longer than killing the processes yourself as was just shown.

Once the renderer starts up properly, it should display something like: ![alt text](https://github.com/mfirlefyn/flightmare/blob/master/docs/init_plac_screenshot.png "Initial placement")

Be sure to click the renderer's screen and press "h" to toggle the visibility of the nest cube. Otherwise you generated images will include the cube. The terminal will start displaying some fields indicating information about the Flightmare-renderer communications. Such a field looks like:
"
frame_id: 1
Rendering environment
Handling environment output
Create message
unpack message metadata
parse metadata
{"pub_vehicles":[{"collision":false,"lidar_ranges":[]}],"frame_id":0}
pub/sub messages not synced
"

Such information fields indicate some information about the ZMQ messaging middleware. The communication for "frame_id 1" has not been initialized yet, which the "pub/sub messages not synced" statement indicates. After a while, between "frame_id 50" and "frame_id 60", the statement disappears and is replaced by a "Feed image data to RGB Camera" statement. At this point, the communication between the Flightmare camera program and the Unity renderer has been established. The 6 subsequent numbers you see appearing indicate the first pixel value of all the images it is accumulating. Let the program continue to print.

Unity has no inherent capability to simulate catadioptric imaging systems or fisheye lenses. Therefore, the camera program takes care of this with a hard-coded pixel mapping from 6 perspective images to a single catadioptric image. The mapping is taken from the [OmniSCV package](https://github.com/Sbrunoberenguel/OmniSCV). The OmniSCV code base is written in Python, so the essential mapping matrix is imported in Flightmare and the camera program contains a part where the Python code is ported to Flightmare's C++.

Starting from "frame_id 101" the program starts indicating that it is saving the perspective images and the last line per field indicates the X,Y position of the simulated quadcopter. You will see that the quadcopter on the renderer screen starts to move and trace a learning trajectory. This can be any trajectory as specified in the camera program. You can observe saved perspective images in the hidden ros folder by running:
```console
nautilus ~/.ros
```

The omni-directional images are saved in a separate folder on your catkin workspace:
```console
nautilus ~/catkin_ws/src/flightmare/flightros/src/camera/images
```

Note that the omni-directional images are grey-scale. The training images are taken as single channel because this limits the weights that need to be trained by the net.

Once the omni-directional images are generated and saved, the simulated learning flight is completed and your terminal will indicate that the outbound flight trajectory can commence. In a real robot, odometry is used to accumulate a home vector. During the outbound trajectory, the home vector will drift due to effects such as wind. This was simplified in this simulation by just guiding the quadcopter with the "awsd" keys on your keyboard and rotating with the "qe" keys. Be sure to have the renderer's window as the active window on your UI by clicking it first. Once the quadcopter completes its outbound flight phase, the drift is simulated by having a perturbed nest location. In other words, we choose the drift that has been accumulated manually. This drift needs to be set in the renderer's source code, which will be explained in its respective [reposity](https://github.com/mfirlefyn/flightmare_unity).

When you want to switch the flight phase from outbound to inbound, simply press the "enter" key once. Again, be sure to have the renderer's window active before pressing "enter". Your terminal will display that the inbound flight phase has commenced and you see the quadcopter fly back to the learning area in a straight line. 

Once the quadcopter has reached the learning area, it will hover in the same place, you can double check the quad's position and rotational vector in the terminal output. Again, make sure the window is active and press "enter" once to start evaluating the images taken on your current location. The program will generate the omni-directional image at your current location, but will there. The terminal will display: 
"
Constructing omni-directional images
Quad Position: {539.349976}, {578.260010}
Quad Rotation: {-0.279110}, {-0.000000}, {-0.000000}, {-0.960260}
Saving omni-directional evaluation picture
Sending message to bearingnet to evaluate available image.
Receiving Home Vector
"

Now, the camera program is waiting on a response from the Python program that feeds the taken omni-directional image through the neural net to get a home vector estimate and sends the output back to the Flightmare simulation to make a step in the estimated home direction. Before the evaluation can be done and the homing achieved, you will need to install the necessary software using this repo (LINK NEEDS TO BE INSERTED!).

You have tested your Flightmare setup and if everything is as expected, you can continue to install the Python program to train and evaluate the network. The Python program is designed to run in a terminal along side the camera program. During your outbound trajectory, you can wait to press the "enter" key such that the neural network has sufficient time to train. Once the training is completed, it will wait for the message from the camera program to start evaluating. The Unity renderer code does not need to be adjusted as long as you do not need to change the perturbed nest location in the learning area (location for the quad to return to in the inbound flight phase) or if you do not need a different simulation environment.

#### Some Extras
In the "~/.ros" directory there is also a "positions.txt" and "rotations.txt" file that logs all of the positions and rotations the quad has gone through during the simulation. You can parse these files in order to plot the trajectory of the simulated quad. Just be sure to clear these files every time you run the simulation since it simply appends the positions of the new simulation to the file.