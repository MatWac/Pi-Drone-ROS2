import rclpy
import serial
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Int8MultiArray

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.subscription = self.create_subscription(
            Int8MultiArray,
            'motor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        datas = ','.join(str(data) for data in msg.data) + '\n'
        self.ser.write(datas.encode())
        self.get_logger().info(datas)

def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()