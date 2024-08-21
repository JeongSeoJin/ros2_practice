import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 시리얼 포트 설정
        self.timer = self.create_timer(1.0, self.read_serial_data)  # 1초마다 데이터 읽기

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Received: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
