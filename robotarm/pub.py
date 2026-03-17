import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
from serial_monitor import getData

class PendulumPublisher(Node):
    last_theta = 0
    last_y = 0
    last_target_pos = 0
    def __init__(self):
        super().__init__('pendulum_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/pendulum_target', 10)
        # Publish at 20Hz (0.05s)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.prev_time = time.time()
        self.last_state = None
        self.get_logger().info('Pendulum Publisher started. Publishing to /pendulum_target')

    def timer_callback(self):

        # + is towards us
        # - is away from us
        # x_rot_v, theta = getData()


        # target/home angles: 60, -93, -53, -34, 30

        # Motion parameters
        center_x = -0.3
        center_y = 0.0
        center_z = 0.4
        amplitude = 0.1 # meters
        frequency = 0.55 # Hz (0.25 cycles per second -> 4 seconds per full cycle)
        threshold = 0.1
        arm_len = .58 # meters
  
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

        ang_vel = d_theta / dt

        


        tan_v = arm_len * ang_vel
        x_v = math.cos(cur_theta) * tan_v



        # THIS SEEMS TO BE WORKING, SPEED NEEDS TO BE CALCULATED CAREFULLY THO
        
        speed = 0.02
        sign = 0
        y = 0
        dist = 0.2

        if abs(x_v) > threshold:
            try:
                sign = x_v/abs(x_v)
                speed = abs(x_v) * 1.2
            except:
                pass
            
            y = dist * sign * 1
        else:
            y = self.last_y
            
        # print("ang vel: ", ang_vel, ", cur theta: ", cur_theta)
        print("x_v:", x_v)

        self.last_y = y


        if abs(y - self.last_target_pos) > 0.1:
            msg = Float64MultiArray()
            # Format: [x, y, z, velocity]
            msg.data = [center_x, y, center_z, speed]
            self.publisher_.publish(msg)
        self.last_target_pos = y

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




        # tan_v = arm_len * x_rot_v
        # x_v = math.cos(theta) * tan_v


        # guess_pos = center_y
        # speed = 0
        # if abs(x_v) > threshold:

        #     guess_pos = center_y + x_v
        #     speed = x_v / 4
            
        # print(guess_pos)


        # Switch between endpoints instead of streaming sine wave
        # period = 1.0 / frequency
        # phase = (t % period) / period
        
        # if phase < 0.5:
        #     state = 1
        #     y = center_y + amplitude
        # else:
        #     state = -1
        #     y = center_y - amplitude
        
        # # Only publish on state change
        # if state == self.last_state:
        #     return
        # self.last_state = state
        
        # # Calculate speed to reach destination in half period (plus margin)
        # dist = 2 * amplitude
        # duration = period / 2.0
        # speed = (dist / duration) * 1.2 # 20% speed margin to ensure arrival