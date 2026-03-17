import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
from serial_monitor import getData

class PendulumPublisher(Node):
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

    speed = 0.

    #state machine
    max_seen_vel = 0
    #rest, ext1, capdata, ext2
    state = "rest"


    def rest(self, vel):
        if abs(vel)>self.threshold:
            self.state = "ext1"

    def ext1(self, vel):
        if abs(vel)>self.threshold:
            self.state = "capdata"

    def capdata(self, vel):
        if abs(vel) > self.threshold:
            if abs(vel) > abs(self.max_seen_vel):
                self.max_seen_vel = vel
        else:
            if abs(self.max_seen_vel)> self.threshold:
                self.state = "ext2"
            else:
                self.state = "rest"


    def ext2(self, vel, theta):

        y = self.arm_len*math.sin(theta) *0.8
        y = min(max(y, -self.y_max_reach), self.y_max_reach)
        msg = Float64MultiArray()
        # Format: [x, y, z, velocity]
        msg.data = [self.center_x, y, self.center_z, self.speed]
        self.publisher_.publish(msg)
        self.state = "ext1"



    def __init__(self):
        super().__init__('pendulum_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/pendulum_target', 10)
        # Publish at 20Hz (0.05s)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.prev_time = time.time()
        self.last_state = None
        self.get_logger().info('Pendulum Publisher started. Publishing to /pendulum_target')

    def timer_callback(self):
        # zero arm and await input
        if not self.started:
            # time.sleep(0.1)
            # msg = Float64MultiArray()
            # msg.data = [self.center_x, 0, self.center_z, self.speed]
            # self.publisher_.publish(msg)
            input("press enter to begin balancing")
            print("\nbegun balancing")
            self.started = True

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

        print("dt:", x_v)

        match self.state:
            case "rest":
                self.rest(x_v)
            case "ext1":
                self.ext1(x_v)
            case "ext2":
                self.ext2(x_v, cur_theta)
            case "capdata":
                self.capdata(x_v)



        


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
