#!/usr/bin/env python

import rclpy
from std_msgs.msg import String
from .conversions import *
from rclpy.node import Node
from pydispatch import dispatcher
import zmq
from threading import Thread


class GDP_Client():
    def __init__(self, gdp_proxy):
        self._publishers = {}
        self._subscribers = {}
        self.gdp_proxy = gdp_proxy
        
        self.context = zmq.Context()
        print ("Connecting to server...")
        self.socket = self.context.socket(zmq.PUB)
        #  Get the reply.
        thread = Thread(target = self.receive, args = ())
        thread.start()
        
    def publisher(self, topic_name, message_type):
        if topic_name in self._publishers:
            publisher = self._publishers.get(topic_name)
            publisher.usage += 1
        else:
            print('Advertising topic {} for publishing'.format(topic_name))
            publisher = _Publisher(self, topic_name, message_type)
            self._publishers[topic_name] = publisher

        #TODO: it should advertise the topic on global data plane
        return publisher
    
    def unregister_publisher(self, topic_name):
        """Stop advertising on the given topic.
        Args:
            topic_name (str): The ROS topic name.
        """
        if topic_name in self._publishers:
            print('Stop advertising topic {} for publishing'.format(topic_name))
            del self._publishers[topic_name]
            
    def subscriber(self, topic_name, message_type, cb):
        subscriber = _Subscriber(self, topic_name, cb)
        if topic_name in self._subscribers:
            self._subscribers.get(topic_name).get(
                'subscribers').append(subscriber)
        else:
            print('Sending request to subscribe topic {}'.format(topic_name))
            self.send(json.dumps({
                'op': 'subscribe',
                'topic': topic_name,
                'type': message_type
            }))
            self._subscribers[topic_name] = {}
            self._subscribers[topic_name]['subscribers'] = [subscriber]
        #TODO: it should advertise the topic on global data plane
        return subscriber

    def unsubscribe(self, subscriber):
        """Remove a callback subscriber from its topic subscription list.
        If there is no callback subscribers in the subscription list.
            It will unsubscribe the topic.
        Args:
            subscriber (_Subscriber): A subscriber with callback function
                that listen to the topic.
        """
        topic_name = subscriber.topic_name
        if topic_name not in self._subscribers:
            return
        subscribers = self._subscribers.get(topic_name).get('subscribers')
        if subscriber in subscribers:
            subscribers.remove(subscriber)
        if len(subscribers) == 0:
            print('Sending request to unsubscribe topic {}'.format(topic_name))
            del subscribers[:]
            self.send(json.dumps({
                'op': 'unsubscribe',
                'topic': topic_name
            }))
            del self._subscribers[topic_name]

    def send(self, message):
        port = "5559"
        self.socket.connect ("tcp://localhost:%s" % port)
        print ("Send: " + message)
        self.socket.send (("%s %s" % ("", message)).encode('utf-8'))

        
    def receive(self):
        port = "5560"
        # Socket to talk to server
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect ("tcp://localhost:%s" % port)
        topicfilter = b""
        socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        while True:
            message = socket.recv().decode()
            print("Received message: ", message)
            data = json.loads(message)
            if (data.get("op") == "subscribe"):
                print("Attempt to subscribe to a local topic")
                self.gdp_proxy.create_new_local_topic(data.get("topic"), data.get("type"))
            if data.get("op") == "unsubscribe":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "advertise":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "unadvertise":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "publish":
                dispatcher.send(signal=data.get('topic'), message=data.get('msg'))
            if data.get("op") == "unadvertise":
                pass            


class _Publisher(object):
    def __init__(self, gdp_client, topic_name, message_type):
        """Constructor for _Publisher.
        Args:
            rosbridge (ROSBridgeClient): The ROSBridgeClient object.
            topic_name (str): The ROS topic name.
            message_type (str): The ROS message type, such as std_msgs/String.
            queue_size (int): The queue created at bridge side for re-publishing.
                Defaults to 1.
        """
        self._gdp_client = gdp_client
        self._topic_name = topic_name
        self._usage = 1

        self._gdp_client.send(json.dumps({
            'op': 'advertise',
            'topic': topic_name,
            'type': message_type,
        }))

    @property
    def usage(self):
        return self._usage

    @usage.setter
    def usage(self, value):
        self._usage = value

    def publish(self, message):
        """Publish a ROS message
        Args:
            message (dict): A message to send.
        """
        self._gdp_client.send(json.dumps({
            'op': 'publish',
            'topic': self._topic_name,
            'msg': message
        }))

    def unregister(self):
        """Reduce the usage of the publisher. If the usage is 0,
        unadvertise this topic."""
        self._usage -= 1
        if self._usage <= 0:
            self._gdp_client.unregister_publisher(self._topic_name)
            self._gdp_client.send(json.dumps({
                'op': 'unadvertise',
                'id': self._advertise_id,
                'topic': self._topic_name
            }))


class _Subscriber(object):
    def __init__(self, gdpclient, topic_name, cb=None):
        """Constructor for _Subscriber.
        Args:
            gdpclient (gdpclient): The gdpclient object.
            topic_name (str): The ROS topic name.
            cb (function): A function will be called when a message is
                received on that topic.
        """
        self._gdp_client = gdpclient
        self._topic_name = topic_name
        self._cb = cb
        if callable(self._cb):
            dispatcher.connect(self._cb, signal=topic_name)

    @property
    def topic_name(self):
        return self._topic_name

    def unregister(self):
        """Remove the current callback function from listening to the topic,
        and from the rosbridge client subscription list
        """
        if callable(self._cb):
            dispatcher.disconnect(self._cb, signal=self._topic_name)
        self._gdp_client.unsubscribe(self)
        
class GDP_Proxy(Node):
    def __init__(self):
        super().__init__('gdp_proxy')

        # topics
        self.remote_topics =[]  # [["chatter", 'std_msgs/String']]
        self.local_topics = [["topic", 'std_msgs/String']]
        self.rate_hz = 1
        self.check_if_msgs_are_installed()

        self.initialize()

    def initialize(self):
        # connect to GDP infrastructure
        self.client = GDP_Client(self)
        
        # connect the topics 
        self._instances = {'topics': [], 'services': []}
        for rt in self.remote_topics:
            if len(rt) == 2:
                topic_name, topic_type = rt
                local_name = topic_name
            elif len(rt) == 3:
                topic_name, topic_type, local_name = rt
            self.create_new_remote_topic(topic_name, topic_type, local_name)

        for lt in self.local_topics:
            if len(lt) == 2:
                topic_name, topic_type = lt
                remote_name = topic_name
            elif len(lt) == 3:
                topic_name, topic_type, remote_name = lt
            self.create_new_local_topic(topic_name, topic_type, remote_name)

    # Topics being published remotely to expose locally
    def create_new_remote_topic(self, topic_name, topic_type, local_name=""):
        if local_name == "":
            local_name = topic_name
        print("create new remote topic to expose locally: ", topic_name, " ", topic_type)

        class RemoteNode(Node):
             def __init__(self):
                super().__init__('gdp_publisher_' +topic_name)
                self.publisher_ = self.create_publisher(get_ROS_class(topic_type)
                                        , local_name, 10)

        remote_node = RemoteNode()

        cb_r_to_l = self.create_callback_from_remote_to_local(topic_name,
                                                                  topic_type,
                                                                  remote_node.publisher_)
        subl = self.create_subscribe_listener(topic_name,
                                                  topic_type,
                                                  cb_r_to_l)
        rospub.impl.add_subscriber_listener(subl)

        self._instances['topics'].append(
                {topic_name:
                 {'rospub': rospub,
                  'bridgesub': None}
                 })
        
    # Topics being published locally to expose remotely
    def create_new_local_topic(self, topic_name, topic_type, remote_name=""):
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

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


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
        
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            #TODO: check if there is a remote topic "sync param"
            #self.sync_params()
            r.sleep()
    
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
                print(
                    "{} could not be found in the system.".format(topic_type))

        for lt in self.local_topics:
            if len(lt) == 2:
                _, topic_type = lt
            elif len(lt) == 3:
                _, topic_type, _ = lt
            if not is_ros_message_installed(topic_type):
                print(
                    "{} could not be found in the system.".format(topic_type))


def main(args=None):
    rclpy.init(args=args)
    proxy = GDP_Proxy()
    rclpy.spin(proxy)
    proxy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()