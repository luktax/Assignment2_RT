import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class distance_check(Node):
    def __init__(self):
        super().__init__('distance_check')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback, 10)
        self.threshold = 0.5
    def listener_callback(self, msg):
        #if min(msg.ranges) < self.threshold
        pass

def main(args=None):
    rclpy.init(args=args)
    distance_check = distance_check()
    rclpy.spin(distance_check)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()