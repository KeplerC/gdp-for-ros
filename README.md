

# To run 

Step 1: set up TUN TAP interface `tunros`
```
# terminal 1
sudo python3 virtual_interface.py (the processing logic)

# terminal 2 
# setup the interface 
sudo ip link set tunros up
# dummy to trick ros to think it's an interface
sudo ip addr add 10.0.0.1 peer 10.0.0.1 dev tunros 
```

Step 2: run ROS

Start two terminals. 

```
# publisher 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://`pwd`/cyclonedds.xml
ros2 run demo_nodes_cpp talker

# subscriber
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://`pwd`/cyclonedds.xml
ros2 run demo_nodes_cpp listener
```


