#!/usr/bin/env python

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from pydispatch import dispatcher
from threading import Thread

from .conversions import *
from .gdp_for_ros import *
from .utils import * 


# hardcode several things 
# TODO: change it with cli args

class GDP_Client():
    def __init__(self, gdp_proxy, switch_ip, switch_name):
        self._publishers = {}
        self._subscribers = {}
        self.gdp_proxy = gdp_proxy
        
        self.switch_ip = switch_ip 
        self.switch_gdpname = int.from_bytes(bytes.fromhex(switch_name), "big")

        # register itself with the switch
        local_ip = get_local_ip()
        self.local_ip = local_ip
        local_gdpname = generate_gdpname(local_ip)
        self.local_gdpname = local_gdpname
        register_proxy(local_ip, switch_ip, local_gdpname, self.switch_gdpname)

        #  Get the reply thread
        self.data_assembler = DataAssembler(local_gdpname, local_ip, switch_ip)
        thread = threading.Thread(target=start_sniffing, args=(lambda packet: self.data_assembler.process_packet(packet),))
        thread.start()
        
    def publisher(self, topic_name, message_type, metadata):
        if topic_name in self._publishers:
            publisher = self._publishers.get(topic_name)
            publisher.usage += 1
        else:
            print('Advertising topic {} for publishing'.format(topic_name))
            publisher = _Publisher(self, topic_name, metadata, message_type, 
                            self.local_ip, self.local_gdpname, self.switch_ip)
            self._publishers[topic_name] = publisher

        return publisher
    
    def unregister_publisher(self, topic_name):
        """Stop advertising on the given topic.
        Args:
            topic_name (str): The ROS topic name.
        """
        if topic_name in self._publishers:
            print('Stop advertising topic {} for publishing'.format(topic_name))
            del self._publishers[topic_name]
            
    def subscriber(self, topic_name, topic_metadata, message_type, cb):
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

            # register with gdp
            sync_rt_info(topic_name, topic_metadata, False, self.local_ip, self.local_gdpname, self.switch_ip)
            # connect_self_to_topic(talker_topic_gdp_name, False, self.local_ip, self.local_gdpname, self.switch_ip)
        
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
        print("generic send not implemented")
        print("message to be sent: " + message)

    def receive(self):
        # Socket to talk to server
        # context = zmq.Context()
        # socket = context.socket(zmq.SUB)
        # socket.connect ("tcp://localhost:%s" % zmq_recv_port)
        # topicfilter = b""
        # socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
        while True:
            uid_and_message = self.data_assembler.message_queue.get()
            print(uuid_and_message, flush=True)
            message = socket.recv().decode()
            print("Received message: ", message)
            data = json.loads(message)
            if (data.get("op") == "subscribe"):
                print("Attempt to subscribe to a local topic")
                self.gdp_proxy.create_new_local_topic(data.get("topic"), data.get("type"))
            if data.get("op") == "publish":
                dispatcher.send(signal=data.get('topic'), message=data.get('msg'))
                
            if data.get("op") == "unsubscribe":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "advertise":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "unadvertise":
                print("dynamically adding new topic not allowed")
                pass
            if data.get("op") == "unadvertise":
                print("dynamically unsubscribing new topic not allowed")
                pass            


class _Publisher(object):
    def __init__(self, gdp_client, topic_name, topic_metadata, message_type,
                local_ip, local_gdpname, switch_ip):
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

        self.local_ip = local_ip 
        self.local_gdpname = local_gdpname
        self.switch_ip = switch_ip 

        # self._gdp_client.send(json.dumps({
        #     'op': 'advertise',
        #     'topic': topic_name,
        #     'type': message_type,
        # }))

        # publish on gdp
        self.topic_gdpname_int = sync_rt_info(topic_name, topic_metadata, True, local_ip, local_gdpname, switch_ip)

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
        message = json.dumps({
            'op': 'publish',
            'topic': self._topic_name,
            'msg': message
        })
        push_message_to_remote_topic(self._topic_name, 
                hex(self.topic_gdpname_int)[2:], 
                self.local_ip, self.local_gdpname, 
                self.switch_ip, message)

        # be able to send over to socket
        # self._gdp_client.send(message)

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
    def __init__(self, config_file_path = "./src/gdp-for-ros/config.json"):
        super().__init__('gdp_proxy')

        # topics
        self.read_in_config(config_file_path)
        self.rate_hz = 1
        self.check_if_msgs_are_installed()

        self.initialize()

    def read_in_config(self, config_file_path):
        with open(config_file_path, "r") as f:
            config_str = f.read()
        config = json.loads(config_str)
        self.ip_switch_publisher = config["switch_ip"]
        self.name_switch_publisher =  config["switch_gdpname"]
        self.local_topics = []
        self.remote_topics = []
        self.remote_topics_metadata = {}
        self.local_topics_metadata = {}
        
        topics = config["topics"]#json.loads(config["topics"])
        for topic in topics:
            print(topic)
            topic_name = topic
            topic_info = topics[topic_name]
            if topic_info["type"] == "local":
                self.local_topics.append([topic_name, topic_info["message_type"]])
                self.local_topics_metadata[topic_name] = topic_info["metadata"]
            elif topic_info["type"] == "remote":
                self.remote_topics.append([topic_name, topic_info["message_type"]])
                self.remote_topics_metadata[topic_name] = topic_info["metadata"]
            else:
                print("topic type unknown: ", topic_info["type"])

        print(self.local_topics, self.remote_topics,self.remote_topics_metadata, self.local_topics_metadata )
            
    def initialize(self):
        # connect to GDP infrastructure
        self.client = GDP_Client(self, self.ip_switch_publisher, self.name_switch_publisher)
        
        # connect the topics 
        self._instances = {'topics': []}
        for lt in self.local_topics:
            if len(lt) == 2:
                topic_name, topic_type = lt
                remote_name = topic_name
            elif len(lt) == 3:
                topic_name, topic_type, remote_name = lt
            self.create_new_local_topic(topic_name, self.local_topics_metadata[topic_name], topic_type, remote_name)

        for rt in self.remote_topics:
            if len(rt) == 2:
                topic_name, topic_type = rt
                local_name = topic_name
            elif len(rt) == 3:
                topic_name, topic_type, local_name = rt
            self.create_new_remote_topic(topic_name, self.remote_topics_metadata[topic_name], topic_type, local_name)


    # Topics being published remotely to expose locally
    def create_new_remote_topic(self, topic_name, topic_metadata, topic_type, local_name=""):
        if local_name == "":
            local_name = topic_name
        print("create new remote topic to expose locally: ", topic_name, " ", topic_type)

        rospub = self.create_publisher(get_ROS_class(topic_type), "gdp/" + local_name, 10)

        cb_r_to_l = self.create_callback_from_remote_to_local(topic_name,
                                                                  topic_type,
                                                                  rospub)
        bridgesub = self.client.subscriber(
                                topic_name,
                                topic_metadata,
                                topic_type,
                                cb_r_to_l)

        self._instances['topics'].append(
                {topic_name:
                 {'rospub': rospub,
                  'bridgesub': bridgesub}
                 })
        
    # Topics being published locally to expose remotely
    def create_new_local_topic(self, topic_name, topic_metadata, topic_type, remote_name=""):
        if remote_name == "":
            remote_name = topic_name
        print("create new local topic to expose remote: ", topic_name, " ", topic_type)
        bridgepub = self.client.publisher(remote_name, topic_type, topic_metadata)

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
            msg = from_dict_to_ROS(message, topic_type)
            rospub.publish(msg)
        return callback_remote_to_local

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
