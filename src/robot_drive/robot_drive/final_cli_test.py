# DOESN"T WORK ------------- EXPERIMENTATION PHASE
# run in new terminal from /home/deepracer:
# cd project_ws && source install/setup.bash && ros2 run robot_drive cli

## Works fine as stand alone with srv but unable to configure lidar with it

## This is the client setup
from deepracer_interfaces_pkg.srv import LidarConfigSrv
import sys
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
import numpy as np

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LidarConfigSrv, 'lidar_config')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LidarConfigSrv.Request()

    def send_request(self):
        self.req.use_lidar = True
        self.req.min_angle = -45.0
        self.req.max_angle = 45.0
        self.req.num_values = 64
        self.req.min_distance = 0.15000000596046448
        self.req.max_distance = 1.0
        self.req.clipping_distance = 1.0
        self.req.num_sectors = 64
        self.req.preprocess_type = 0

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # runs once and exits
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of service call for max angle of "%f" = "%s"' % (minimal_client.req.max_angle, response.error))

    minimal_client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
