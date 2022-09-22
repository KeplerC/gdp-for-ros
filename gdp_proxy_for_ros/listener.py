import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/gdp/benchmark',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'benchmark_echo', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        echo_msg = String()
        echo_msg.data = msg.data[:40]
        self.get_logger().info('I heard: "%s"' % echo_msg.data[:40])
        self.publisher_.publish(echo_msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
