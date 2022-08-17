### What the code does: 


### Setup Env
Install ROS2 properly and create a local ROS workspace. 
Install pythons depenencies by
```
pip3 install PyDispatcher 
```
A working repo is  hosted at `gdpmobile1/fog_ws/src/`.

### To run 

Step 1: import the ROS env 
```
cd ~/fog_ws
source install/setup.bash
```
This needs to be done on every new terminals.

Step 2: build
```
colcon build
```
Whenever there is code changes, a new build is required.

Step 3: Run
First start the pseudo gdp infrastructure by 
```
python3 fake_gdp_infrastructure.py
```
then run the sample talker and listener by
```
ros2 run gdp_proxy_for_ros talker
```
and
```
 ros2 run gdp_proxy_for_ros listener
```
The talker publishes to the topic named `topic` and the listener subscribes to `/gdp/topic`. 

Then run
```
`ros2 run gdp_proxy_for_ros proxy
```
that bridges the `topic` and `gdp/topic` together. The messages in `topic` are serialized and propaged through `fake_gdp_infrastructure.py` implemented by zmq. (TODO:Jiachen, replace with the real router). 

### Note

1. The code adopts the code from draft PR from https://github.com/uos/rospy_message_converter/pull/56/, because ROS2 lacks message serialization schemes. 
2. several convienient commands 
```
colcon build && ros2 run gdp_proxy_for_ros proxy
ros2 topic list
ros2 topic echo /gdp/topic
ros2 topic info /topic
```