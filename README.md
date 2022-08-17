
### Setup Env

```
pip3 install PyDispatcher 
```

### To run 

The repo is hosted at `gdpmobile1/fog_ws/src/`.

Step 1: import the ROS env 
```
cd ~/fog_ws
source install/setup.bash
```

Step 2: build
```
colcon build
```

Step 3: Run
(TODO)
```
ros2 run gdp_proxy_for_ros talker
ros2 run gdp_proxy_for_ros listener
ros2 run gdp_proxy_for_ros proxy
```