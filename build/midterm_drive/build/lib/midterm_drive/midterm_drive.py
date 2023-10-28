# imports
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
import time

class MinimalPubSub(Node):
    def __init__(self):
        # inside the node
        super().__init__('minimal_pubsub')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.wheel_subscriber = self.create_subscription(LaserScan, "/rplidar_ros/scan", self.sub_callback, 10)
        self.wheel_subscriber

    def sub_callback(self,msg):
        wheel_msg = ServoCtrlMsg()
        wheel_msg.angle = 0.0
        self.get_logger().info('Distance(m): "%s"' % min(msg.ranges))
        if min(msg.ranges) > 1.3:
            wheel_msg.throttle = 0.8
        else:
            wheel_msg.throttle = 0.0

        self.wheel_publisher.publish(wheel_msg)
        #time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub = MinimalPubSub()
    rclpy.spin(minimal_pubsub)

    minimal_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
