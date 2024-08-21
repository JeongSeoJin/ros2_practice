import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CenterPointSubscriber(Node):

    def __init__(self):
        super().__init__('center_point_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'object_center',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if len(msg.data) == 4:
            cx, cy, width_mm, height_mm = msg.data
            self.get_logger().info(f'Received center point: ({cx}, {cy}), Size: ({width_mm} mm, {height_mm} mm)')
        else:
            self.get_logger().info('Received unexpected data format.')

def main(args=None):
    rclpy.init(args=args)
    node = CenterPointSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
