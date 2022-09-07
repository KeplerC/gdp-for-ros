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
2. Register a proxy that only has a publisher, and advertise a topic called "helloworld". On gdpmobile5, run
    ```
    cd gdp-router-client-proxy/
    . venv/bin/activate
    ```
    and then, using the gdpname showed on gdpmobile2 (IP address is 128.32.37.82),
    ```
    sudo env "PATH=$PATH" python gdp_for_ros.py 128.32.37.82 [gdpname of gdpmobile2] 1 123123123
    ```

3. Register a proxy that only has a subscriber, and connect itself as a subscriber to the topic "helloworld". On gdpmobile6, run
    ```
    cd gdp-router-client-proxy/
    . venv/bin/activate
    ```
    and then, using the gdpname of gdpmobile4 and the gdpname
    of the topic "helloworld" (this should show on the gdpmobile5 terminal starting with "0x" prefix, remember not to include this hex prefix), run
    ```
    sudo env "PATH=$PATH" python gdp_for_ros.py 128.32.37.42 [gdpname of gdpmobile4] 0 [gdpname of "helloworld" topic]
    ```
    By ths time, gdpmobile3 tab should show that there is a topic called "helloworld" in the GDP, and this topic has one publisher and one subscriber

4. Push a message to topic "helloworld", just hit any key on gdpmobile5



