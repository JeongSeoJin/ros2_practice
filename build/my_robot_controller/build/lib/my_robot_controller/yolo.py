#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from tutorial_interfaces import Num

class publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0

    def timer_callback(self):
        # self.i = 0
        msg = Num()                                           # CHANGE
        msg.num = self.x                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE

def main(args=None):
    rclpy.init(args=args)

    pub = publisher()

    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()