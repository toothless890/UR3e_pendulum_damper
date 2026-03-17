import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
from serial_monitor import getData


def sign(val):
    if val == 0:
        return 1
    else:
        return abs(val)/val

class PendulumPublisher(Node):

    center_x = -0.3
    center_y = 0.0
    center_z = 0.4
    amplitude = 0.1 # meters
    frequency = 0.55 # Hz (0.25 cycles per second -> 4 seconds per full cycle)
    threshold = 0.05
    arm_len = .58 # meters
    speed = 0.6

    last_theta = 0
    last_y = 0
    last_target_pos = 0

    rest_thresh = 0.001
    ext_thresh = 0.1
    start_swing_thresh = 0
    max_y_magnitude = 0.3

    running = False

    vel_prev_sign = 1

    # STATE MACHINE
    
    # vars
    
    max_vel = 0
    
    state = "rest" # rest, ext1, ext2, startswing, capdata

    def rest(self, vel):
        if abs(vel) > self.rest_thresh:
            self.state = "ext1"
            self.max_vel = 0

    def rest2(self, vel):
        if abs(vel) > 0.25*self.max_vel:
            self.state = "ext1"
            self.max_vel = 0
    
    def ext1(self, vel):
        if self.vel_prev_sign != sign(vel):
            self.state = "capdata"
            self.vel_prev_sign = sign(vel)
    
    def capdata(self, vel, cur_theta):
        if sign(vel) == self.vel_prev_sign:
            if abs(vel) > abs(self.max_vel):
                self.max_vel = vel
        else:
            if abs(self.max_vel) < self.ext_thresh:
                self.state = "rest"
            else:

                y_target = self.arm_len * math.sin(cur_theta) * 0.2

                y_target = -self.clamp(y_target, -self.max_y_magnitude, self.max_y_magnitude)
                print(self.max_vel)

                if (abs(self.max_vel) > 2.5):
                    y_target *=2
                    time.sleep(0.3865*0.8)
                # elif (abs(self.max_vel)> 1):
                #     y_target *=1.3

                #     time.sleep(0.3865*0.7)

                else:
                    y_target = -y_target
                    self.max_vel *=0.7
                msg = Float64MultiArray()

                # msg.data = [self.center_x, -y_target, self.center_z, self.clamp(abs(self.max_vel) * 0.5, -self.speed, self.speed)]
                # # Format: [x, y, z, velocity]
                # self.publisher_.publish(msg)

                msg.data = [self.center_x, -y_target, self.center_z, self.clamp(abs(self.max_vel) * 0.5, -self.speed, self.speed)]
                self.publisher_.publish(msg)
                # time.sleep(0.2)
                # time.sleep(math.sqrt(abs(self.max_vel)) / 10)
                # time.sleep()
                # self.last_target_pos = self.y
                self.state = "rest2"



    def clamp(self, val, minval, maxval):
        return min ( max(val, minval), maxval)


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

        if not self.running:
            input("press enter to begin balancing")
            self.running = True

  
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

        


        tan_v = self.arm_len * ang_vel
        x_v = math.cos(cur_theta) * tan_v
            
        # print("ang vel: ", ang_vel, ", cur theta: ", cur_theta)
        # print("dt:", x_v)

        match self.state:
            case "rest":
                self.rest(x_v)
            case "ext1":
                self.ext1(x_v)
            case "capdata":
                self.capdata(x_v, cur_theta)
            case "rest2":
                self.rest2(x_v)
            # case "_":
        # print(self.state)
                

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