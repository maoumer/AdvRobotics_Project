# CSCI 4302/5302 Midterm
# Drive straight while there is no wall within 1m in front of robot
# Team Maleficent - Andy Ho, Matt Ward, Mohammed Adib Oumer, Rameez Wajid
# Fall 2023

## for manual start (line 6), for manual stop (line 7) - using CLI from workspace directory
# ros2 topic pub --once /ctrl_pkg/servo_msg deepracer_interfaces_pkg/msg/ServoCtrlMsg "{throttle: 0.8, angle: 0.0}"
# ros2 topic pub --once /ctrl_pkg/servo_msg deepracer_interfaces_pkg/msg/ServoCtrlMsg "{throttle: 0.0, angle: 0.0}"

# run in new terminal:
# cd project_ws && source install/setup.bash && ros2 run robot_drive pubsub

# imports
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
import time
import numpy as np

class MinimalPubSub(Node):
    def __init__(self):
        # inside the node
        super().__init__('minimal_pubsub')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.wheel_subscriber = self.create_subscription(LaserScan, "/rplidar_ros/scan", self.sub_callback, 10)
        self.wheel_subscriber
        # self.i = 0
        

    def sub_callback(self, msg):
        wheel_msg = ServoCtrlMsg()
        wheel_msg.angle = 0.0
        
        rec_msg = np.array(msg.ranges)
        rec_msg[np.isinf(rec_msg)] = rec_msg.min() + 12.0
        front_msg = np.concatenate((rec_msg[-120:], rec_msg[:120])) #lidar data representing front data
        min_front_distance = min(front_msg)
        if (min_front_distance > 1.0):
            wheel_msg.throttle = 0.5
        else:
            wheel_msg.throttle = 0.0
        
        self.get_logger().info('Closest Distance(m): "%s"\n' % (min_front_distance))
        
        '''
        # self.i+=1
        # if self.i == 20 or self.i == 25 or self.i == 29:
        #     # for i,m in enumerate(msg.ranges):
        #     #     if m > 0.9:
        #     #         ms[i] = 0
        #     #     else: 
        #     #         ms[i] = m
        #     ms = np.array(ms)
        #     min_indx = np.argmin(ms)
        #     sect1, sect2 = ms[min_indx-32:min_indx+32], ms[min_indx-64:min_indx+64]
        #     self.get_logger().info('Distance(m): "%s","%s", "%s", \n"%s","%s", \n"%s","%s",\n\n' % (len(ms), min_indx, ms[min_indx], np.mean(sect1), sect1, np.mean(sect2), sect2))
        # for i,value in enumerate(msg.ranges):
        #     self.get_logger().info('index: "%s"' % i)
        #     self.get_logger().info('STUFF: "%s"' % value)
        
        # self.get_logger().info('Distance(m): "%s", "%s"\n' % (len(msg.ranges), min(msg.ranges)))
        # if min(msg.ranges) > 0.9:
        #     wheel_msg.throttle = 0.0
        # else:
        #     wheel_msg.throttle = 0.0
        '''

        self.wheel_publisher.publish(wheel_msg)



def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub = MinimalPubSub()
    rclpy.spin(minimal_pubsub)

    minimal_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
