# DOESN"T WORK COMPLETELY ------------- EXPERIMENTATION PHASE
# CSCI 4302/5302 Midterm
# Team Maleficent - Andy Ho, Matt Ward, Mohammed Adib Oumer, Rameez Wajid
# Fall 2023

# run in new terminal:
# cd project_ws && source install/setup.bash && ros2 run robot_drive drive_around
# ros2 topic pub --once /ctrl_pkg/servo_msg deepracer_interfaces_pkg/msg/ServoCtrlMsg "{throttle: 0.0, angle: 0.0}"

# imports
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
import time
import numpy as np
# from simple_pid import PID

class MinimalPubSub(Node):
    def __init__(self):
        # inside the node
        super().__init__('minimal_pubsub')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.wheel_subscriber = self.create_subscription(LaserScan, "/rplidar_ros/scan", self.sub_callback, 10)
        self.wheel_subscriber
        self.i = 0
        self.start_time  = time.time()

    def sub_callback(self, msg):
        wheel_msg = ServoCtrlMsg()
        # wheel_msg.angle = 0.0
        turning_right = False
        sector_ranges = [120,168,200,238,279,315,347,387,456]
        self.i += 1

        # get reading
        received_msg = np.array(msg.ranges)
        # To get a specific range from the lidar in degrees, type the degree range that you want multiplied by this number
        deg_conv = len(received_msg) / 360
        # sectors = []
        # for i in range(0, len(received_msg), 8):
        #     partition = received_msg[i:i+8]
        #     sectors.append(partition)
        # self.get_logger().info('BEFORE: Curr Lidar Read: "%s"\n' % (max(received_msg)))
        # received_msg[received_msg == np.inf] = max(received_msg)
        # self.get_logger().info('After: Curr Lidar Read: "%s"\n' % (max(received_msg)))
        # updated_received_msg = np.ones_like(received_msg) * 2
        
        received_msg[np.isinf(received_msg)] = min(received_msg) + 6.0
        # updated_received_msg = received_msg[np.isinf(received_msg, updated_received_msg)]
        # np.where(updated_received_msg == 1,min(received_msg) + 3.0, updated_received_msg)
        self.get_logger().info('Curr Lidar Read: "%s"\n' % (min(received_msg)))

        min_front_distance = min(np.min(received_msg[:20]), np.min(received_msg[-20:]))
        # turn left: (wheel angle = positive)
        left_msg = received_msg[120:168]
        right_msg = received_msg[387:456]
        moving_avg_left = []
        moving_avg_right = []
        window_size = 10
        i = 0 
        while i < len(left_msg) - window_size +1:
            # Store elements from i to i+window_size
            # in list to get the current window
            window = left_msg[i : i + window_size]
        
            # Calculate the average of current window
            window_average = round(sum(window) / window_size, 2)
            
            # Store the average of current
            # window in moving average list
            moving_avg_left.append(window_average)
            
            # Shift window to right by one position
            i += 1
        i = 0
        while i < len(right_msg) - window_size +1:
            # Store elements from i to i+window_size
            # in list to get the current window
            window = right_msg[i : i + window_size]
        
            # Calculate the average of current window
            window_average = round(sum(window) / window_size, 2)
            
            # Store the average of current
            # window in moving average list
            moving_avg_right.append(window_average)
            
            # Shift window to right by one position
            i += 1
        # left_msg = np.mean(sectors[2]) #190 before 
        # right_msg = np.mean(sectors[6])
        # straight_on_msg = np.concatenate((received_msg[-50:],received_msg[:50])) #was -110:100
        # straight_right_msg = received_msg[420:440]
        criteria = right_msg-left_msg #np.argmax(np.bincount(right_msg))  - np.argmax(np.bincount(left_msg)) #
        cruise = 0.5
        turn_speed = 0.45
        turn = 0.3

        if min_front_distance < 0.5:
            wheel_msg.throttle = 0.0
            wheel_msg.angle = 0.0
        elif (criteria < -0.2): # left steering
            wheel_msg.throttle = turn_speed
            wheel_msg.angle = turn
        elif (criteria > 0.2): # right steering
            wheel_msg.throttle = turn_speed
            wheel_msg.angle = -turn
        else:
            wheel_msg.throttle = cruise
            wheel_msg.angle = 0.0

        self.get_logger().info('Closest Distance(m): "%s", "%s", "s%", "%s"\n' % (min_front_distance,right_msg,left_msg,len(received_msg)))
        self.get_logger().info('Curr Throttle: "%s"\n' % (wheel_msg.throttle))
        self.get_logger().info('Curr Angle: "%s"\n' % (wheel_msg.angle))

        self.wheel_publisher.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub = MinimalPubSub()
    rclpy.spin(minimal_pubsub)

    minimal_pubsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
