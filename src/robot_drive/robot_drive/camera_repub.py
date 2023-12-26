# DOESN"T WORK ALWAYS ------------- EXPERIMENTATION PHASE
# CSCI 4302/5302 Final
# Publish camera data since RViz was not working
# Team Maleficent - Andy Ho, Matt Ward, Mohammed Adib Oumer, Rameez Wajid
# Fall 2023

# run in new terminal:
# cd project_ws && source install/setup.bash && ros2 run robot_drive camera

# imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import time
import numpy as np

class MinimalPubSub(Node):
    def __init__(self):
        # inside the node
        super().__init__('camera_repub')
        self.camera_subscriber = self.create_subscription(Image, "/camera_pkg/display_mjpeg", self.sub_callback, 10)
        self.camera_pub = self.create_publisher(Image, "/image", 10)

        self.frame_num = 0

        

    def sub_callback(self, msg):
        self.frame_num += 1

        msg.header.frame_id = "id_" + str(self.frame_num)
        # msg.header.stamp = self.get_clock().now()

        self.camera_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub = MinimalPubSub()
    rclpy.spin(minimal_pubsub)

    minimal_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
