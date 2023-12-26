# DOESN"T WORK COMPLETELY ------------- EXPERIMENTATION PHASE
# Helpful commands
# ros2 interface show deepracer_interfaces_pkg/srv/LidarConfigSrv
# ros2 service call /sensor_fusion_pkg/configure_lidar deepracer_interfaces_pkg/srv/LidarConfigSrv "{'min_angle':-60.0}"

# run in new terminal from /home/deepracer:
# cd project_ws && source install/setup.bash && ros2 run robot_drive srv

## Works fine as stand alone but unable to configure lidar with it

# This is the service setup
from deepracer_interfaces_pkg.srv import LidarConfigSrv

import rclpy
from rclpy.node import Node
import time
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(LidarConfigSrv, 'lidar_config', self.lidar_config_callback)

    def lidar_config_callback(self, request, response):
        start_time = time.time()
        response = LidarConfigSrv.Response()
        time_taken = (time.time() - start_time)*1e6
        self.get_logger().info('Incoming request with min angle "%s" received. Time taken: "%s microsec"' % (request.min_angle, time_taken))
        
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


########################################################################################
########################################################################################
'''
## This the ServiceClient together. Does nto seem to work!
# from deepracer_interfaces_pkg.srv import LidarConfigSrv
# import sys
# import rclpy
# from rclpy.node import Node
# import time
# from std_msgs.msg import String
# import numpy as np

# class MinimalServiceClient(Node):

#     def __init__(self):
#         super().__init__('minimal_srvcli')
#         self.sub_node = rclpy.create_node('sub_node')
#         self.srv = self.create_service(LidarConfigSrv, 'lidar_config', self.lidar_config_callback)
#         self.cli = self.sub_node.create_client(LidarConfigSrv, 'lidar_config')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')

#     def lidar_config_callback(self, request: LidarConfigSrv.Request, response: LidarConfigSrv.Response):
#         req = LidarConfigSrv.Request()
#         req.use_lidar = True
#         req.min_angle = -45
#         req.max_angle = 45
#         req.num_values = 64
#         req.min_distance = 0.15000000596046448
#         req.max_distance = 1
#         req.clipping_distance = 1
#         req.num_sectors = 64
#         req.preprocess_type = 0
        
#         future = self.cli.call_async(req)
#         rclpy.spin_until_future_complete(self.sub_node, future)
#         if future.result() is not None:
#             result: LidarConfigSrv.Response = future.result()
#             response.error = result.error
#             self.get_logger().info('Result of service call with min angle {}: {}'.format(request.min_angle, response.error))
#             return response
#         else:
#             self.get_logger().error('Exception while calling service: {}'.format(future.exception()))


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_srvcli = MinimalServiceClient()

#     rclpy.spin(minimal_srvcli)

#     # minimal_srvcli.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
'''
