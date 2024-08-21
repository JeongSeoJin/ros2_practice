import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataSubscriberNode(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'serial_data',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
