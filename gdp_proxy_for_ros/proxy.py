#!/usr/bin/env python

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from pydispatch import dispatcher
from threading import Thread
from .conversions import *
import socket

from .rosbridge_client import ROSBridgeClient

"""
Server to expose locally and externally
topics, services and parameters from a remote
roscore to a local roscore.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

yaml_config = '''
# ROSbridge websocket server info
rosbridge_ip: 192.168.1.31
rosbridge_port: 9090
# Topics being published in the robot to expose locally
remote_topics: [
                    ['/joint_states', 'sensor_msgs/JointState'], 
                    ['/tf', 'tf2_msgs/TFMessage'],
                    ['/scan', 'sensor_msgs/LaserScan']
                    ]
# Topics being published in the local roscore to expose remotely
local_topics: [
                    ['/test1', 'std_msgs/String'],
                    ['/closest_point', 'sensor_msgs/LaserScan']
                    ]
# Services running in the robot to expose locally
remote_services: [
                    ['/rosout/get_loggers', 'roscpp/GetLoggers']
                    ]
# Services running locally to expose to the robot
local_services: [
                    ['/add_two_ints', 'beginner_tutorials/AddTwoInts']
                    ]
# Parameters to be sync, they will be polled to stay in sync
parameters: ['/robot_description']
parameter_polling_hz: 1'''


class ROSduct(Node):
    def __init__(self):
        super().__init__('gdp_proxy')
        # ROSbridge
        self.rosbridge_ip = "20.84.87.187"
        if self.rosbridge_ip is None:
            rospy.logerr('No rosbridge_ip given.')
            raise Exception('No rosbridge_ip given.')
        self.rosbridge_port = 9090
        print("Will connect to ROSBridge websocket: ws://{}:{}".format(
            self.rosbridge_ip, self.rosbridge_port))

        # Topics
        # TODO: check if topic types are installed, if not, give a warning
        self.remote_topics = [['/chatter_1', 'std_msgs/String']]
        print("Remote topics: " + str(self.remote_topics))
        self.local_topics = [['/chatter', 'std_msgs/String']]
        print("Local topics: " + str(self.local_topics))

        self.parameters = []
        self.check_if_msgs_are_installed()

        self.initialize()

    def initialize(self):
        """
        Initialize creating all necessary bridged clients and servers.
        """
        connected = False
        while not connected:
            try:
                self.client = ROSBridgeClient(
                    self.rosbridge_ip, self.rosbridge_port)
                connected = True
            except socket.error as e:
                print(
                    'Error when opening websocket, is ROSBridge running?')
                print(e)

        # We keep track of the instanced stuff in this dict
        self._instances = {'topics': []}
        for lt in self.local_topics:
            if len(lt) == 2:
                topic_name, topic_type = lt
                remote_name = topic_name
            elif len(lt) == 3:
                topic_name, topic_type, remote_name = lt
            self.create_new_local_topic(topic_name, topic_type, remote_name)

        for rt in self.remote_topics:
            if len(rt) == 2:
                topic_name, topic_type = rt
                local_name = topic_name
            elif len(rt) == 3:
                topic_name, topic_type, local_name = rt
            self.create_new_remote_topic(topic_name, topic_type, local_name)


    # Topics being published remotely to expose locally
    def create_new_remote_topic(self, topic_name, topic_type, local_name=""):
        if local_name == "":
            local_name = topic_name
        print("create new remote topic to expose locally: ", topic_name, " ", topic_type)

        rospub = self.create_publisher(get_ROS_class(topic_type), local_name, 10)

        cb_r_to_l = self.create_callback_from_remote_to_local(topic_name,
                                                                  topic_type,
                                                                  rospub)
        bridgesub = self.client.subscriber(
                                topic_name,
                                topic_type,
                                cb_r_to_l)

        self._instances['topics'].append(
                {topic_name:
                 {'rospub': rospub,
                  'bridgesub': bridgesub}
                 })
        
    # Topics being published locally to expose remotely
    def create_new_local_topic(self, topic_name,  topic_type, remote_name=""):
        if remote_name == "":
            remote_name = topic_name
        print("create new local topic to expose remote: ", topic_name, " ", topic_type)
        bridgepub = self.client.publisher(remote_name, topic_type)

        cb_l_to_r = self.create_callback_from_local_to_remote(topic_name,
                                                            topic_type,
                                                            bridgepub)

        rossub = self.create_subscription(
                        get_ROS_class(topic_type),
                        topic_name,
                        cb_l_to_r,
                        10)

        self._instances['topics'].append(
                {topic_name:
                 {'rossub': rossub,
                  'bridgepub': bridgepub}
                 })


    def create_callback_from_remote_to_local(self, topic_name,
                                             topic_type,
                                             rospub):
        # Note: argument MUST be named 'message' as
        # that's the keyword given to pydispatch
        def callback_remote_to_local(message):
            print("Remote ROSBridge subscriber from topic " +
                           topic_name + ' of type ' +
                           topic_type + ' got data: ' + str(message) +
                           ' which is republished locally.')
            # Only convert and publish with subscribers

            msg = from_dict_to_ROS(message, topic_type)
            rospub.publish(msg)
        return callback_remote_to_local

    def create_callback_from_local_to_remote(self,
                                             topic_name,
                                             topic_type,
                                             bridgepub):
        def callback_local_to_remote(message):
            print("Local subscriber from topic " +
                           topic_name + ' of type ' +
                           topic_type + ' got data: ' + str(message) +
                           ' which is republished remotely.')
            dict_msg = from_ROS_to_dict(message)
            bridgepub.publish(dict_msg)
        return callback_local_to_remote

    def create_subscribe_listener(self,
                                  topic_name,
                                  topic_type,
                                  cb_r_to_l):
        # We create a SubscribeListener that will
        # create a rosbridge subscriber on demand
        # and also unregister it if no one is listening
        class CustomSubscribeListener(rospy.SubscribeListener):
            def __init__(this):
                super(CustomSubscribeListener, this).__init__()
                this.bridgesub = None

            def peer_subscribe(this, tn, tp, pp):
                # Only make a new subscriber if there wasn't one
                if this.bridgesub is None:
                    print(
                        "We have a first subscriber to: " + topic_name)
                    this.bridgesub = self.client.subscriber(
                        topic_name,
                        topic_type,
                        cb_r_to_l)
                    for idx, topic_d in enumerate(self._instances['topics']):
                        if topic_d.get(topic_name):
                            self._instances['topics'][idx][topic_name]['bridgesub'] = this.bridgesub
                            break

            def peer_unsubscribe(this, tn, num_peers):
                # Unsubscribe if there isnt anyone left
                if num_peers < 1:
                    print(
                        "There are no more subscribers to: " + topic_name)
                    self.client.unsubscribe(this.bridgesub)
                    this.bridgesub = None
                    # May be redundant if it's actually a reference to this.bridgesub already
                    for idx, topic_d in enumerate(self._instances['topics']):
                        if topic_d.get(topic_name):
                            self._instances['topics'][idx][topic_name]['bridgesub'] = None
                            break
        return CustomSubscribeListener()

    def check_if_msgs_are_installed(self):
        """
        Check if the provided message types are installed.
        """
        for rt in self.remote_topics:
            if len(rt) == 2:
                _, topic_type = rt
            elif len(rt) == 3:
                _, topic_type, _ = rt
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))

        for lt in self.local_topics:
            if len(lt) == 2:
                _, topic_type = lt
            elif len(lt) == 3:
                _, topic_type, _ = lt
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))


    def sync_params(self):
        """
        Sync parameter server in between
        external and local roscore (local changes
        are not forwarded).
        """
        for param in self.parameters:
            if type(param) == list:
                local_param = param[1]
                param = param[0]
            else:
                local_param = param
            # Get remote param
            remote_param = self.client.get_param(param)
            if remote_param != self.last_params[param]:
                rospy.set_param(local_param, remote_param)
                self.last_params[param] = remote_param

    # def spin(self):
    #     """
    #     Run the node, needed to update the parameter server.
    #     """
    #     r = rospy.Rate(self.rate_hz)
    #     while not rospy.is_shutdown():
    #         self.sync_params()
    #         r.sleep()


def main():
    rclpy.init(args=None)
    proxy = ROSduct()
    rclpy.spin(proxy)
    proxy.destroy_node()
    rclpy.shutdown()