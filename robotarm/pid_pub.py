import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
from serial_monitor import getData
from geometry_msgs.msg import PoseStamped

import pid_gui

def clamp( val, minval, maxval):
        return min ( max(val, minval), maxval)

class PendulumPublisher(Node):
    last_target = 0

    arm_y_pos = 0
    center_x = -0.3
    center_y = 0.0
    center_z = 0.4
    amplitude = 0.1 # meters
    frequency = 0.55 # Hz (0.25 cycles per second -> 4 seconds per full cycle)
    threshold = 0.1

    arm_len = .58 # meters
    last_theta = 0

    y_max_reach = 0.3

    started = False

    speed = 0.3


    center_buffer_size = 30
    center_buffer = []
    for x in range(center_buffer_size):
        center_buffer +=[0]
    center_buffer_index = 0

    balance_buffer_size = 30
    balance_buffer = []
    for x in range(balance_buffer_size):
        balance_buffer += [0]
    balance_buffer_index =0

    # def pid(self, pid_params, state_curr, state_goal, integral_buffer, integral_index, integral_buffer_size = 100):
    #     proportional = pid_params[0]*(state_goal-state_curr)

    #     integral_index = (integral_index + 1) % integral_buffer_size

    #     integral = 0
    #     for i in range (integral_buffer_size):
    #         integral += integral_buffer[i]
        
        
    def pidCentering(self, pos, lin_vel, goal, integral_buffer, integral_index, integral_buffer_size, dt):
        proportional = (goal-pos)
        derivative = lin_vel
        
        integral_index = (integral_index + 1) % (integral_buffer_size)

        integral = 0
        for i in range (integral_buffer_size):
            integral += integral_buffer[i]
        pid = pid_gui.get_pid1()
        
        result = pid[0]*proportional+pid[1]*integral+pid[2]*derivative

        integral_buffer[integral_index] = pos * dt

        # print(pos*dt)
        # print(proportional, integral, derivative)
        return result
    
    def pidBalancing(self, theta, ang_vel, goal, integral_buffer, integral_index, integral_buffer_size, dt) :
        proportional = (goal-theta)
        derivative = ang_vel
        
        integral_index = (integral_index + 1) % (integral_buffer_size)

        integral = 0
        for i in range (integral_buffer_size):
            integral += integral_buffer[i]

        pid = pid_gui.get_pid2()
        result = pid[0]*proportional+pid[1]*integral+pid[2]*derivative

        integral_buffer[integral_index] = theta * dt
        # print(proportional, integral, derivative)

        return result


    def __init__(self):
        super().__init__('pendulum_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/pendulum_target', 10)
        self.tcp_pose = self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.tcp_pose_callback, 10)
        
        # Publish at 20Hz (0.05s)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.prev_time = time.time()
        self.last_state = None
        self.get_logger().info('Pendulum Publisher started. Publishing to /pendulum_target')

    def tcp_pose_callback(self, msg):
        self.arm_y_pos = float(-msg.pose.position.y-0.0022)
        # print(round(self.arm_y_pos, 3))

    def timer_callback(self):
        # zero arm and await input
        # if not self.started:
        #     input("press enter to begin balancing")
        #     print("\nbegun balancing")
        #     self.started = True

        # calc delt time
        cur_time = time.time()
        dt = cur_time - self.prev_time
        self.prev_time = cur_time
        # print("delta time: ", dt)

        data = getData()
        # cur_theta = data[1] % (2*math.pi)
        cur_theta = data[1]
        d_theta = self.last_theta - cur_theta
        self.last_theta = cur_theta

        # y_pos=self.arm_len*math.sin(cur_theta)
        ang_vel = d_theta / dt
        tan_v = self.arm_len * ang_vel
        lin_velocity = math.cos(cur_theta) * tan_v




        target_pos = 0
        target_pos += pid_gui.get_jog()

        balance_target = self.pidBalancing(cur_theta, ang_vel, 0, self.balance_buffer, self.balance_buffer_index, self.balance_buffer_size, dt)
        balance_compensation = self.arm_len*math.sin(balance_target) 
        self.balance_buffer_index +=1
        target_pos += balance_compensation
        
        # print(cur_theta)
        center_target =  self.pidCentering(target_pos, self.speed, 0, self.center_buffer, self.center_buffer_index, self.center_buffer_size, dt)
        self.center_buffer_index +=1
        target_pos += center_target

        thresh = pid_gui.get_thresh()

        if (abs(self.last_target - target_pos) > thresh):
            print(target_pos)
            print(self.arm_y_pos)
            print()
            # print(target_pos)
            msg = Float64MultiArray()
            msg.data = [self.center_x, clamp(target_pos, -0.3, 0.3 ), self.center_z, clamp(abs(target_pos-self.arm_y_pos)*3, 0, 0.4 )] #
            self.publisher_.publish(msg)
            self.last_target = target_pos
            time.sleep(0.2)


        


def main(args=None):
    rclpy.init(args=args)
    node = PendulumPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
