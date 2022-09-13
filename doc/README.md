# Global Data Plane (GDP) Pub-Sub Demo 

## Overview
A publisher advertises a topic called "helloworld" to the Global Data Plane(GDP), at the same time claiming it is a publisher of this topic. 

A subscriber sends a "connect me" request to the GDP to be added to the "helloworld" topic. 

The publisher then pushes some message to the topic hosted in the GDP, and the subscriber receives the message.

## Limitation
- The routing part of the network is yet to be integrated. Currently, all the topic information are simply stored in an entity called "access point" in the GDP. It behaves like a ROS master to manage all the topic related things in a centralized manner.

- Any message pushed to any topic would be received on every registered proxy since we are broadcasting the message to all proxies. Security is a big TODO. Also, this broadcasting approach leaves the filtering responsibility (from a proxy's standpoint, it will need to figure out if a received message is from a topic that it is subcribed to) to the proxy itself.

## Steps to run this demo
This demo needs 5 NUCs: gdpmobile2-6. gdpmobile5 as a publisher, gdpmobile6 as a subscriber. gdpmobile2 and gdpmobile4 as switch. gdpmobile3 as the access point.

1. Set up a GDP network: 
    - On gdpmobile3, starts the access point
        ```
        rungdp
        cd gdp/jiachen_playground
        cargo run -- -m 1
        ```
    - On gdpmobile2 and gdpmobile4, start and register switch to the access point (IP of gdpmobile3 is 128.32.37.41)
        ```
        rungdp
        cd gdp/jiachen_playground
        cargo run -- -m 0 -a 128.32.37.41
        ```
2. The above step generates the gdpname for each switch. Update the `~/fog_ws/src/gdp_proxy_for_ros/proxy.py` L16-20 with the corresponding generated gdpnames. 

    Then Register a proxy that only has a publisher. On gdpmobile5 and 6, run
    ```
    cd fog_ws
    sudo su
    source ./install/setup.bash
    colcon build                        # to build the ROS package 
    ros2 run gdp_proxy_for_ros proxy    # to run the proxy
    ```
    
3. Run ROS application. Start two new terminals, on the terminal with gdpmobile 5, run the talker by
    ```
    cd ~/fog_ws/
    source ./install/setup.sh
    ros2 run gdp_proxy_for_ros talker
    ```
    
    and run the listener on gdpmobile 6 by
    ```
    cd ~/fog_ws/
    source ./install/setup.sh
    ros2 topic list
    ros2 topic echo /gdp/topic
    ```
    

