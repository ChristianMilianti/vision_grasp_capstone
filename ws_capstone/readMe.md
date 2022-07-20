![crop](https://user-images.githubusercontent.com/41469847/141243673-ef70abde-aeb4-469f-b7ca-4c5124223c44.png)


# How to Install

## System

- Linux Ubuntu 18.04 LTS (Bionic Beaver)
- Recommend 30-40GB+ for HD space

## Installation

- [Install ROS Melodic](http://wiki.ros.org/ROS/Installation)

- Clone this repository and activate catkin workspace
```
catkin clean
catkin build
```

- Catkin ws directory:  `vision_grasp_capstone/ws_captsone/`
- Python 2.7 env
- Dependencies
```
sudo apt-get install ros-melodic-gazebo-ros ros-melodic-eigen-conversions ros-melodic-object-recognition-msgs ros-melodic-roslint
```

- Install dependencies from requirements.txt
- Note Christian Milianti * I went into both links and combined both the requirements from  both into one requirements file which can be found in the main folder of this repo
- I then installed requirements with
```
pip install -r requirements.txt
```
# The following below is what I did to try get the installation working. STILL WIP.
 i simply performed  after i installed the requirements from above by going into  
 ```
  /home/christian/ROS_Projects/vision_grasp_capstone/ws_capstone/src/mvp_grasp-master
 ```
Then I ran the following command.
```
     rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```
also think i installed the newer version of torch again but im not sure, since my gpu is new i dont think the old torch python 2 versions are supporting the newer cuda versions, so im going to run it on cpu only by editing ggcnn as above.

```
    pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113
```
 then i got this error ImportError: No module named builtins, so i did this
    ```
    pip2 install future
    ```

When I tried running the object detection node i got the following errors:
- ImportError: No module named skimage.transform , I solved it by doing:
    ```
    pip2 install scikit-image
    ```
- IOError: /home/christian/ROS_Projects/vision_grasp_capstone/ws_capstone/src/mvp_grasp-master/yolov3_pytorch_ros-master/models/yolov3_tiny_custom_2000.weights not found.
    - This is because they haven't uploaded their custom weights to github... to use other weights change what weights and config you want to use in their detector.launch file. they had the config for their custom yolo v3 tiny model, but not the weights.

When I tried running the capstone scheduler I faced the error
 - no module named moveit_commander
 so i installed move it with
 ``` 
 sudo apt install ros-melodic-moveit
```

 My robot wouldnt move in Gazebo and I received these erros when running the first launch command
  - Action client not connected: ur5e_arm_controller/follow_joint_trajectory
  - Action client not connected: gripper2_controller/follow_joint_trajectory

  So I tried installing the universal robot move it package by doing the following, if you don't do this, the robot in gazebo won't move.

 ```
# These are the instructions I used below.
-------------------------------------------------------------

#inside of src in your catkin ws

git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git

cd ..


rosdep update

rosdep install --rosdistro melodic --ignore-src --from-paths src


catkin build

source devel/setup.bash

#Done. See generic instructions below.

-------------------------------------------------------------

# Generic instructions.
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

## how to import models into gazebo

Solution taken from - https://answers.gazebosim.org//question/6347/downloading-models-fails/
- You can download these models yourself from here  https://github.com/osrf/gazebo_models . I simply downloaded all of them >700mb.
- And then put these models in /usr/share/gazebo-[VERSION]/models. 
- You may need to chmod and change the folder permissions to allow you to add the models
- to do this i found where my models folder was and then chmod 777 that folder only, then i was able to simply drag and drop my extracted models into that folder.
```
$ cd /usr/share/gazebo-9
$ sudo chmod 700 models
```
- For example, my Gazebo version is 9, so I put all these model files inside /usr/share/gazebo-9/models
- Then once you run the spawning and deleting objects commands in the instructions further below it should work


## Capstone Scheduler - 

Main program to run the robot. Main `go()` function is the current program running which currently integrates both the YOLO and GGCNN network callbacks and some basic filtering of the YOLO bounding boxes. The movement of the robot is controlled by the moveit library. 

## Run Project

1. Launches the robot model in Gazebo and Rviz:
```
roslaunch moveit_config demo_gazebo.launch
```
2. Launches Grasping node:
```
roslaunch ggcnn ggcnn_service.launch
```
3. Launches object detection node:
```
roslaunch yolov3_pytorch_ros detector.launch
```
4. Launches main program and controller:
```
rosrun mvp_grasping capstone_scheduler.py
```

*Currently the project uses a simulated sensor in gazebo. To use the real world D435 use the following command:
```
roslaunch mvp_grasping wrist_realsense.launch
```

Modifications to the code may be required where the camera topic names are defined such as 'ggcnn_service.yaml'

### Spawning and deleting objects in Gazebo
```
#this spawns a coke can and a plastic cup
rosrun mvp_grasping spawn_models.py

#this deletes all models in the scene (but i need to look at it as it seems to not delete some)
rosrun mvp_grasping delete_models.py

#this lets you select from a list of items
rosrun mvp_grasping objects_task1.py

#this spawns a few items
rosrun mvp_grasping objects_task2.py


```




# Info on paper/code used:
## GGCNN 
- By D. Morrison et al. 
    - https://github.com/dougsm/ggcnn
    - https://github.com/dougsm/mvp_grasp

GGCNN generative grasping and ROS node modified from this research. This code also contains some higher level improvements for grasping implemented in task schedulers and could be worth a look at reintroducing. 
- [GGCNN paper](https://arxiv.org/abs/1804.05172)
- [GGCNN improved grasping](https://arxiv.org/abs/1809.08564)


GGCNN node output:
```
geometry_msgs/Pose pose
float32 width
float32 quality
```
## YOLOv3 

ROS nodes for more later YOLOs are hard to find for python 2.7 so we used YOLOv3 here. This will need to be retrained for any new objects and could be replaced with a segmentation algorithm or better object detector. Currently the bounding boxes are used to estimate the location of the objects. It uses a crude estimation of the height to do this and should be improved to use the orientation of the camera to accurately translate the bounding box coordinates to the robot base frame. The current translation is found in the function transform() in `vision_grasp_capstone\ws_capstone\src\mvp_grasp-master\yolov3_pytorch_ros-master\src\yolov3_pytorch_ros\detector.py`. This works by locking off both the height and orientation of the camera when scanning. 

YOLO node output:
```
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
```


# Contributors
### 2021 Contributors:
- Liam Jemmeson
- Joshua Ten
- Rodney Bakiskan
- Nuwan Devasurendra
- Taha Mohsin Abdul Redha Al Lawati

Further information can be found on github for the respective libraries used regarding installation. There may be some dependencies missing from these lists so please add them here where necessary if you come across anything crucial. Feel free to contact me at ljemmeson@gmail.com to assist with installation, setup or any questions about the project


### 2022 Updates by Christian Milianti:
 ## Ignore the following striked out text below.
 
<s>  When i did the above, I had the following issues running the 2.grasping node

- ImportError: No module named scipy.ndimage
    - I fixed it by running the following 2 commands
    ```
        sudo apt-get install python-scipy
        pip install scipy
    ```
-  ImportError: No module named torch
    - I checked my cuda version with $ nvidia-smi , i then installed torch via following the instructions for pip linux python install found here https://pytorch.org/get-started/locally/

    ```
        pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113
    ```
    - I faced another issue because it still wouldnt find the torch module, I assume its because it was a python 3 package, and it was looking for a python 2 packages, so it i tried installing an older version of torch+cpu
    ```
     pip install torch==1.3.1+cpu torchvision==0.4.2+cpu -f https://download.pytorch.org/whl/torch_stable.html

    ```
    - still didnt work, so i tried editing lines 24-30 of ggcnn.py from /ws_capstone/src/mvp_grasp-master/ggcnn/src/ggcnn/ggcnn.py" and changing it from gpu to cpu by commenting out the gpu lines and uncommenting the cpu lines... this still didnt work

   - ok... scratch everything i just said </s>