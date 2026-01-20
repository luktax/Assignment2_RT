import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import Bool
from assignment2_rt_cpp.msg import CustomMessage

class distance_check(Node):
    def __init__(self):
        super().__init__('distance_check')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback, 10)
        
        self.threshold = 0.5 # Default fallback
        self.safety_pub = self.create_publisher(Bool, '/safety_alert', 10)
        self.custom_pub = self.create_publisher(CustomMessage, '/custom_message', 10)
        
        # Subscribe to parameter events to monitor changes
        self.param_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.param_event_callback,
            10
        )

    def param_event_callback(self, msg):
        # check if event comes from the right node
        for param in msg.changed_parameters:
            if param.name == 'threshold':
                self.threshold = param.value.double_value
                self.get_logger().info(f'Threshold updated to {self.threshold}')

    def listener_callback(self, msg):
        below_threshold = [
            (i, r)
            for i, r in enumerate(msg.ranges)
            if r < self.threshold and r != float('inf') and r == r
        ]

        any_below = len(below_threshold) > 0

        # Publish safety alert always
        msg_bool = Bool()
        msg_bool.data = any_below
        self.safety_pub.publish(msg_bool)

        if any_below:
            # compute closest obstacle and direction
            closest_index, closest_distance = min(below_threshold, key=lambda x: x[1])
            angle = msg.angle_min + msg.angle_increment * closest_index
            if -math.pi/4 <= angle <= math.pi/4:
                direction = 0
            elif math.pi/4 < angle <= 3*math.pi/4:
                direction = 1
            elif -3*math.pi/4 <= angle < -math.pi/4:
                direction = 2
            else:
                direction = 3

            custom_message = CustomMessage()
            custom_message.distance = closest_distance
            custom_message.direction = direction
            custom_message.threshold = self.threshold
            self.custom_pub.publish(custom_message)

            self.get_logger().warn(f'OBSTACLE DETECTED! Range < {self.threshold}')

def main(args=None):
    rclpy.init(args=args)
    node = distance_check()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()