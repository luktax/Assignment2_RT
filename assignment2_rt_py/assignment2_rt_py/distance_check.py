import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import Bool

class distance_check(Node):
    def __init__(self):
        super().__init__('distance_check')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback, 10)
        
        self.threshold = 0.5 # Default fallback
        self.safety_pub = self.create_publisher(Bool, '/safety_alert', 10)
        
        # Subscribe to parameter events to monitor changes
        self.param_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.param_event_callback,
            10
        )

    def param_event_callback(self, msg):
        # check if event comes from the right node
        if msg.node == '/threshold_server':
            for param in msg.changed_parameters:
                if param.name == 'threshold':
                    self.threshold = param.value.double_value
                    self.get_logger().info(f'Threshold updated to {self.threshold}')

    def listener_callback(self, msg):
        # Filter out infinite or NaN values if any, though typically ranges adhere to min/max
        valid_ranges = [r for r in msg.ranges if not (float('inf') == r or float('nan') == r)]
        
        if not valid_ranges:
            return

        # Check if ANY range is below the threshold
        any_below = any(r < self.threshold for r in valid_ranges)
        
        msg_bool = Bool()
        msg_bool.data = any_below
        self.safety_pub.publish(msg_bool)
        
        if any_below:
            self.get_logger().warn(f'OBSTACLE DETECTED! Range < {self.threshold}')
        else:
            # Optional: Log okay state occasionally or just stay silent
            pass

def main(args=None):
    rclpy.init(args=args)
    node = distance_check()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()