# DOESN"T WORK COMPLETELY ------------- EXPERIMENTATION PHASE
# CSCI 4302/5302 Midterm
# Team Maleficent - Andy Ho, Matt Ward, Mohammed Adib Oumer, Rameez Wajid
# Fall 2023

# run in new terminal:
# cd project_ws && source install/setup.bash && ros2 run robot_drive drive_around_open_loop

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
        wheel_msg.angle = 0.0
        turning_right = False
        sector_ranges = [120,168,200,238,279,315,347,387,456]
        self.i += 1

        # get reading
        received_msg = np.array(msg.ranges)
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
        # left_msg = received_msg[120:168]
        # right_msg = received_msg[387:456]
        left_msg = received_msg[80:168] #190 before 
        # straight_on_msg = received_msg[-120:120]
        straight_on_msg = np.concatenate((received_msg[-50:],received_msg[:50])) #was -110:100
        straight_left_msg = received_msg[150:180]
        right_msg = received_msg[400:490]
        criteria = np.mean(right_msg)-np.mean(left_msg) #np.argmax(np.bincount(right_msg))  - np.argmax(np.bincount(left_msg)) #
        cruise = 0.5
        turn_speed = 0.45
        turn = 0.4
        
        # if np.min(straight_left_msg) > np.min(straight_on_msg):
        #     wheel_msg.throttle = 0.4
        #     wheel_msg.angle = 1.0

        if min_front_distance < 0.5 :
            wheel_msg.throttle = 0.0
            wheel_msg.angle = 0.0
        elif ((np.median(right_msg) > 0.95)): # right steering
            wheel_msg.throttle = turn_speed
            wheel_msg.angle = -turn
        # else: #elif (np.min(right_msg) > 0.65 and np.min(right_msg) < 0.95): # go straight
        #     wheel_msg.throttle = cruise
        #     wheel_msg.angle = 0.0
        elif ((np.median(right_msg) < 0.65)): # left steering
            wheel_msg.throttle = turn_speed
            wheel_msg.angle = turn
        elif (0.85 < np.median(right_msg) < 0.95): # go straight
            wheel_msg.throttle = cruise
            wheel_msg.angle = 0.0
        




        # if (np.mean(left_msg)>2.0 and np.mean(right_msg)<1.5): # turn left, positive angle criteria < -0.6
        #     wheel_msg.throttle = 0.4
        #     wheel_msg.angle = 0.4
        # elif (np.mean(right_msg)>2.0 and np.mean(left_msg)<1.5): # turn right criteria > 0.6
        #     wheel_msg.throttle = 0.4
        #     wheel_msg.angle = -0.4
        # else:
        #     # drive forward
        #     if (min_front_distance > 1.0):
        #         wheel_msg.throttle = 0.5
        #         wheel_msg.angle = 0.
        #     elif(min_front_distance < 0.5):
        #         wheel_msg.throttle = 0.0
        #         wheel_msg.angle = 0.
        #     else:
        #         wheel_msg.throttle = 0.4
        #         wheel_msg.angle = 0.

        # turn_speed = 0.38
        # turn = 0.35
        # cruise = 0.45
        # right_msg = received_msg[400:450]
        # # if (len(right_msg) != 0):
        # if (np.min(right_msg)> 0.7): # right steering
        #     wheel_msg.throttle = turn_speed
        #     wheel_msg.angle = -turn
        # else: # go straight
        #     wheel_msg.throttle = cruise
        #     wheel_msg.angle = 0.0

        # if (time.time()-self.start_time > 30):
        #     wheel_msg.throttle = 0.0
        #     wheel_msg.angle = 0.0

        self.get_logger().info('Closest Distance(m): "%s", "%s", "%s"\n' % (min_front_distance,np.min(right_msg),len(received_msg)))
        self.get_logger().info('Curr Throttle: "%s"\n' % (wheel_msg.throttle))
        self.get_logger().info('Curr Angle: "%s"\n' % (wheel_msg.angle))


        
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

        '''
        left_msg = received_msg[160:210]
        right_msg = received_msg[350:400]
        min_left_msg = np.min(left_msg)
        min_right_msg = np.min(right_msg)
        diff = min_left_msg - min_right_msg
        
        
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
