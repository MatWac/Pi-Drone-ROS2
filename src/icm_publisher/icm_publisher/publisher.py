import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3 

import math
from icm20948 import ICM20948

imu = ICM20948()

def getFilteredAngles(pitchGyroFavoring, rollGyroFavoring, pitch, roll, filterUpdateRate):

    pitchFromAccel = 0
    rollFromAccel  = 0

    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

    pitchFromAccel = -math.atan2(-ax , math.sqrt(pow(ay, 2) + pow(az, 2))) * (180/math.pi)	
    rollFromAccel = -math.atan2(ay , math.sqrt(pow(ax, 2) + pow(az, 2))) * (180/math.pi)
	
    pitch = (pitchGyroFavoring) * (pitch + (gy * (1/filterUpdateRate))) + (1.00 - pitchGyroFavoring) * (pitchFromAccel)
    roll = (rollGyroFavoring) * (roll + (gx * (1/filterUpdateRate))) + (1.00 - rollGyroFavoring) * (rollFromAccel)

    return pitch, roll

class ICMPublisher(Node):
    
    def __init__(self):
        super().__init__('icm_publisher')
        self.publisher_ = self.create_publisher(Vector3 , 'angles', 10)
        
        self.updateRate = 10
        self.gyroFavoring = 0.7
        self.pitch = 0
        self.roll = 0

        timer_period = 1/self.updateRate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        msg = Vector3 ()
        self.pitch, self.roll = getFilteredAngles(self.gyroFavoring, self.gyroFavoring, self.pitch, self.roll, self.updateRate)
        msg.y, msg.x = self.pitch, self.roll
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f","%f"' % (msg.x, msg.y))

def main(args=None):
    rclpy.init(args=args)

    icm_publisher = ICMPublisher()

    rclpy.spin(icm_publisher)
    
    icm_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()