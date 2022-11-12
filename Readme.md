## IRL_SLAM
### Info
+ Essay title:
Edge Assisted Multi-user Augmented Reality with a New Approach of Scene Recognition
+ Author:
r08921078 chun-yi chang
### Run
For YOLO and ORB-SLAM2: `
./catkin_ws/open_yolo.sh
`
```bash
source catkin_ws/devel/setup.sh
roslaunch catkin_ws/src/IRL_SLAM/launch/IRL_Server.launch
```
For websocket that connect ROS and Android: `
./catkin_ws/open_ws.sh
`
```bash
source catkin_ws/devel/setup.sh
source catkin_ws/my_env/bin/activate
source catkin_ws/build_cv_bridge.sh
roslaunch catkin_ws/src/IRL_SLAM/launch/IRL_SLAM.launch
```
In the above script, the `build_cv_bridge.sh` script will re-build `cv_bridge` in another folder and then copy files to the target. Otherwise, we will encounter following error when excuting above code:
```
connected loss dynamic module does not define module export function (PyInit_cv_bridge_boost)
```
### System Overview

#### cube slam/line lbd
+ `cube_slam/line_lbd/msg` message structure
+ `cube_slam/detect_3d_cuboid/src` handle messages

#### Virtual_Angle.py
+ set_host start twice result in the `twc['num']` or `host_classes` wrong

#### Depth
+ 220328: replaced the megadepth with glpdepth, no obvious improvemnet in retrieving error.

### Problems Found
+ 220425
    + The depth return to phone seems like the ratio of depth of center point of host image and view image.
    + The depth return to phone after viewer check is too big when using two phones(>10) which should be within 2.
    + The depth return to phone will be compressed to some range, but pixel 6 always get the anchor point on the ground but not desk.

### Main Focus
#### 220315
+ Test other scene, e.g., trees
+ viewer go in from door
+ host and viewer on phone respectively
+ multiple viewers go in from different angle
#### 220305
+ Make the system accommodate multiple user scenario.
+ Decrease the localization error of the placement of the virtual object.

### Future Work
#### 220305
Following statements are extracted from interviewers of brian's master's degree oral exam.
+ This system only use one phone. We simply block the camera to simulate two host and view. The system can be scaled to accomodate multiple phones.
+ This system only work with the scene of the desk setup. The system can be scaled to accomodate different scenario.
+ This system used ORB-SLAM2. We have to check if there is similar function(scene recognition/loop closing/map merge) in latest ORB-SLAM version.
+ The depth information in the system is set to constant. Author have used MegaDepth to obtain the depth information, but failed to archive better result. We have to check if there is better tool to archive this.
### Reference
+ [Python3 and CVBridge Problem](https://blog.csdn.net/qq_33445388/article/details/116034290)