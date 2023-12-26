# Battery service test, PID controller
from deepracer_interfaces_pkg.srv import BatteryLevelSrv
import rclpy
from rclpy.node import Node
import time

# run in new terminal:
# cd project_ws && 
# source install/setup.bash && ros2 run robot_drive battery

# battery status check (-1 if disconnected, 0-11 if connected. 11 full, 0 depleted)
# ros2 service call /i2c_pkg/battery_level deepracer_interfaces_pkg/srv/BatteryLevelSrv

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self.client = self.create_client(BatteryLevelSrv, "/i2c_pkg/battery_level")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        req = BatteryLevelSrv.Request()
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
# def main(args=None):
#     rclpy.init(args=args)

#     minimal_client = BatteryClient()
#     # for i in range(401):
#     response = minimal_client.send_request()
#     minimal_client.get_logger().info('Battery level: "%f"' %(response.level))

#     minimal_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
    

#####################################################################################################
# class PIDController:
#     def __init__(self, k_p, k_i, k_d):
#         self.k_p = k_p
#         self.k_i = k_i
#         self.k_d = k_d
#         self.integral = None
#         self.last_error = None
#         self.last_time = None
#         self.reset()

#     def reset(self):
#         self.integral = 0.0
#         self.last_error = 0.0
#         self.last_time = time.time()
    
#     def step(self, current_position, set_point=0.0):
#         # Get time
#         current_time = time.time()
        
#         # Time elapsed
#         dt = current_time - self.last_time
        
#         # Error
#         error = set_point - current_position
        
#         # Integral
#         self.integral += error * dt
        
#         # Derivative
#         derivative = 0
#         if dt > 0:
#             derivative = (error - self.last_error) / dt            
        
#         # PID formula
#         output = self.k_p * error + self.k_i * self.integral + self.k_d * derivative
        
#         # Update error and time
#         self.last_error = error
#         self.last_time = current_time
        
#         # Return the raw output. Need to map this to smaller values
#         return output
