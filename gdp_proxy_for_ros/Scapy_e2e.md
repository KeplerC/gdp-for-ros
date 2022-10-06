## Overview
The `scapy_demo.py` is a basic echo demo. It has two modes. 
- If run as "sender", it sends 50 packet from gdpmobile5 to gdpmobile6 and expect echo packets to come back. 
- If run as "receiver", it waits packet to come in and simply echo it back.
On the sender side, the RTT is measured using timestamp.

## How to Run
Start the receiver machine first.
On gdpmobile6
```
cd /home/gdpmobile5/fog_ws/src/gdp-for-ros/gdp_proxy_for_ros
sudo python3 scapy_demo.py receiver
```


On gdpmobile5
```
cd /home/gdpmobile5/fog_ws/src/gdp-for-ros/gdp_proxy_for_ros
sudo python3 scapy_demo.py sender
```

