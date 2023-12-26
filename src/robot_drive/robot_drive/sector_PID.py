# DOESN"T WORK COMPLETELY ------------- EXPERIMENTATION PHASE. Tried dividing LiDAR readings with sectors to decide directions
# # CSCI 4302/5302 Final
# Team Maleficent - Andy Ho, Matt Ward, Mohammed Adib Oumer, Rameez Wajid
# Fall 2023

# run in new terminal:
# cd project_ws && 
# source install/setup.bash && ros2 run robot_drive sector_PID

# test throttle and angle once
# ros2 topic pub --once /ctrl_pkg/servo_msg deepracer_interfaces_pkg/msg/ServoCtrlMsg "{throttle: 0.0, angle: 0.0}"

# battery status check (-1 if disconnected, 0-11 if connected. 11 full, 0 depleted)
# ros2 service call /i2c_pkg/battery_level deepracer_interfaces_pkg/srv/BatteryLevelSrv

# imports - project specific
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from sensor_msgs.msg import LaserScan
from robot_drive.relevant_classes import BatteryClient
from simple_pid import PID

# imports - generic
import numpy as np
import time
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

# Three lines to make our compiler able to draw: to plot later
# # import sys
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt

# The driver does right wall follow and uses the LIDAR message from the right and front to loop around
class ForwardDriver(Node):
    def __init__(self):
        # inside the node, create publisher, subscriber
        super().__init__('forward_driver')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.wheel_subscriber = self.create_subscription(LaserScan, "/rplidar_ros/scan", self.sub_callback, 10)
        self.wheel_subscriber

        # default values - only change here
        self.cruise = 0.45
        self.turn_angle = 0.1 # descriptive enough. Negative = Right steer

        # relevant parameters
        self.straight_cruise = self.cruise # relative speed/throttle as it drives straight
        self.turn_cruise = self.cruise*0.95 # relative speed/throttle as it turns right at a hallway
        self.center_point = 1.5 # hallway used is ~3m wide. Change as needed for a course
        self.threshold = 0.15 # distance needed above center point to force turning right
        self.STRAIGHT = True # bool to denote modes - straight or turn
        self.RIGHT = True # bool to denote right wall follow (or left wall follow)
        # battery_check(self)

        # PID right wall follow - straight through the hallway
        self.k_p = 1.0 # pushes robot right reading towards the setpoint with this constant
        self.k_i = 0.0 # steady state error not too relevant in our case
        self.k_d = 0.95 # adjusts rate of reaching goal - slow down if setpoint is close
        self.setpoint = 0.7 # maintain a distance of 0.7m from the wall
        self.pid = PID(self.k_p, self.k_i, self.k_d, setpoint=self.setpoint)
        self.pid.tunings = (self.k_p, self.k_i, self.k_d)
        self.pid.sample_time = 0.001 # in seconds
        self.pid.output_limits = (-0.3, 0.3) # wheel angle adjustments should be minimal (instead of -1 to 1)
        
        # PID right turn - controls turning when the right side LIDAR readings are huge
        self.k_p_turn = 2.0 # pushes robot right reading towards the setpoint with this constant
        self.k_i_turn = 0.0 # steady state error not too relevant in our case
        self.k_d_turn = 1.7 # adjusts rate of reaching goal
        self.setpoint_turn = 0.85 # distance to maintain as robot turns from the wall
        self.pid_turn = PID(self.k_p_turn, self.k_i_turn, self.k_d_turn, setpoint=self.setpoint_turn)
        self.pid_turn.tunings = (self.k_p_turn, self.k_i_turn, self.k_d_turn)
        self.pid_turn.sample_time = 0.001
        self.pid_turn.output_limits = (-0.65, 0.65) # wheel angle adjustments should be a lot more than driving straight
        self.pid_turn.auto_mode = False # safety mode. do not use the turn pid unless we are turning

        self.ctr = 1 # counter
        self.plot_data = [] # store data needed to be plotted
    
    def sub_callback(self, msg):
        wheel_msg = ServoCtrlMsg()
        wheel_msg.angle = 0.0
        # sector_ranges = [120,168,200,238,279,315,347,387,456] # counterclockwise - based on LIDAR overlay on website
        
        # get reading and replace inf values
        received_msg = np.array(msg.ranges)
        received_msg[np.isinf(received_msg)] = min(received_msg) + 12.0
        
        num_partitions = 12
        partition_size = len(received_msg) // num_partitions
        remainder = len(received_msg) % num_partitions

        partitions = []

        # Iterate over the partitions
        start_index = 0
        for i in range(num_partitions):
            end_index = start_index + partition_size + (1 if i < remainder else 0)
            partition = received_msg[start_index:end_index]
            partitions.append(partition)
            start_index = end_index
        
        # get relevant readings - most likely need to be modified with different device/robot
        front_msg = np.concatenate((np.array(partitions[0]), np.array(partitions[11]))) # first and last x = 50 messages
        right_msg = np.concatenate((np.array(partitions[2]), np.array(partitions[3]))) # partitions[2:3]
        left_msg = np.concatenate((np.array(partitions[8]), np.array(partitions[9])))  # partitions[8:9]
        min_front_distance = np.min(front_msg)
        min_right_distance = np.min(right_msg)
        min_left_distance = np.min(left_msg)
        min_distance, multiplier = min_right_distance, 1
        if not self.RIGHT:
            min_distance, multiplier = min_left_distance, -1

        # placeholder variables for readjusting robot when it's too close to crash to a wall from the front
        steer_angle,steer_throttle = 0.1*multiplier, self.turn_cruise 
        turn = True
        if (min_front_distance > 0.65): # if front is open, needs readjusting according to the new robot device
            if self.STRAIGHT: # drive straight mode
                ## Original idea was to place the robot at the center of the hallway
                ## Accounting for left message was adding a lot more uncertainty
                # if (min_distance > 3.0):
                #    min_distance = 2.
                # diff = min_right_distance - min_left_distance
                # self.plot_data.append(diff)

                # set angle and throttle of wheels accordingly
                wheel_msg.angle = self.pid(min_distance)*multiplier # diff
                wheel_msg.throttle = self.straight_cruise

                # fake turn -- needs a lot more work
                # if (min_distance > 1.0 and min_distance < 1.1): #
                #     now = time.time()
                #     while(time.time() - now < 0.08):
                #         wheel_msg.angle = self.turn_angle*1.2*multiplier
                #         wheel_msg.throttle = self.turn_cruise
                #         self.wheel_publisher.publish(wheel_msg)
                #     # wheel_msg.angle = -self.turn_angle*0.9*multiplier
                #     # wheel_msg.throttle = self.turn_cruise
                #     wheel_msg.angle = 0.5*self.pid(min_distance)*multiplier # diff
                #     wheel_msg.throttle = self.turn_cruise
                
                ## if right reading is large (indicating a turn)
                if (min_distance > self.center_point+self.threshold):
                    # Switch to TURN RIGHT mode
                    self.STRAIGHT = False
                    self.pid.auto_mode = False      # disable distance keeping PID
                    self.pid_turn.auto_mode = True    # enable turning PID
                    turn = True
                    # wheel_msg.angle = -self.turn_angle*multiplier
                    # wheel_msg.throttle = self.turn_cruise
            
            else: # turning mode
                now = time.time()
                while (time.time()-now < 0.08 and turn):
                    wheel_msg.angle = -0.4*multiplier
                    wheel_msg.throttle = self.turn_cruise
                    self.wheel_publisher.publish(wheel_msg)
                    turn = False
                wheel_msg.angle = self.pid_turn(min_distance)*multiplier
                wheel_msg.throttle = self.turn_cruise

                # if right reading is small (indicating end of turning)
                if (min_distance < self.center_point+self.threshold):
                    # Switch to DRIVE STRAIGHT mode
                    self.STRAIGHT = True            # go back to driving straight
                    self.pid.auto_mode = True       # enable distance keeping PID
                    self.pid_turn.auto_mode = False   # disable turning PID
                    turn = True
            
            # update steering variables
            steer_angle, steer_throttle = wheel_msg.angle, wheel_msg.throttle
        
        else: # if front is close to a wall
            now = time.time()
            while (time.time()-now < 0.25):
                wheel_msg.angle = -min(steer_angle*2.5,1.0)*multiplier
                wheel_msg.throttle = -min(steer_throttle*1.2, 1.0)
                self.wheel_publisher.publish(wheel_msg)

            wheel_msg.angle = min(steer_angle*2.0,1.0)*multiplier
            wheel_msg.throttle = min(steer_throttle*1.2, 1.0)
        
        self.get_logger().info('Closest Distance(m) [F,R,L]: "%s", "%s", "%s"\n' % (min_front_distance, min_right_distance, min_left_distance))
        self.get_logger().info('Curr Throttle, Angle: "%s", "%s"\n' % (wheel_msg.throttle, wheel_msg.angle))
        self.get_logger().info('Length: "%s"\n' % (len(received_msg)))
        self.wheel_publisher.publish(wheel_msg)


def battery_check(driver):
    # check battery status and update throttle
    client = BatteryClient() # create a client
    response = client.send_request() # request service for battery level
    if (response.level > 6):
        driver.straight_cruise = driver.cruise
        driver.turn_cruise = driver.cruise*0.9
    elif (response.level <= 6 and response.level > 3):
        driver.straight_cruise = driver.cruise*1.1
        driver.turn_cruise = driver.cruise*0.9*1.1
    else:
        driver.straight_cruise = 0.0
        driver.turn_cruise = 0.0
        driver.get_logger().info('Battery too low or not connected. Switch/Connect Battery')
    client.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    forward_driver = ForwardDriver()
    rclpy.spin(forward_driver)
    forward_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

