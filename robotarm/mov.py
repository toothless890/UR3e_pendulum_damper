#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import ikpy.chain

class URMoveToConfig(Node):

    # tcp_pose = None

    def __init__(self):
        super().__init__('ur_move_to_config')
        
        # The controller topic to publish to.
        # 'scaled_joint_trajectory_controller' is the default for the UR driver.
        # If using simulation without the scaled controller, you might need 
        # 'joint_trajectory_controller/joint_trajectory'.
        topic_name = '/scaled_joint_trajectory_controller/joint_trajectory'
        
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # Load the chain from URDF
        # NOTE: You must provide the correct path to your robot's URDF file.
        # The active_links_mask assumes a standard UR chain: Base(Fixed), 6 Joints, Tip(Fixed)
        self.urdf_path = 'ur3e.urdf' 
        self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path, active_links_mask=[False, False, True, True, True, True, True, True, False])

        # Define the robot base position in the world frame [x, y, z]
        self.robot_base_pos = np.array([0.0, 0.0, 0.0])

        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        

        # Subscription for target commands: Expects [x, y, z, velocity]
        # x, y, z in meters; velocity in m/s
        self.target_sub = self.create_subscription(Float64MultiArray, '/pendulum_target', self.target_callback, 10)

    def joint_state_callback(self, msg):
        # Map joint names to positions to ensure correct order
        # UR joints: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        try:
            name_map = {n: p for n, p in zip(msg.name, msg.position)}
            self.current_joints = [name_map[n] for n in order]
        except KeyError:
            pass

    

    def target_callback(self, msg):
        if self.current_joints is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return

        if len(msg.data) < 4:
            self.get_logger().error("Invalid target msg. Expected [x, y, z, velocity]")
            return

        # Parse target
        target_pos_world = np.array(msg.data[:3])
        # Convert world target to robot-relative target
        target_pos = target_pos_world - self.robot_base_pos
        velocity = msg.data[3]
        if velocity <= 0: velocity = 0.1 # Safety default

        # Target orientation: Pointing towards +Y direction
        # The matrix columns represent the Tool frame axes [X, Y, Z] in World coordinates.
        # Col 2 (Z-axis) is the "pointer". We set it to [0, 1, 0] (World +Y).
        # Col 0 (X-axis) is arbitrary orthogonal. We set it to [0, 0, 1] (World +Z).
        # Col 1 (Y-axis) is Z cross X. We set it to [1, 0, 0] (World +X).
        target_orientation = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])

        # Seed IK with current robot state
        # Pad current 6 joints to match links in chain (indices 2-7 are active)
        ik_seed = [0.0] * len(self.chain.links)
        
        # Map current joints to active links (2-7) and clamp to bounds to prevent ValueError
        for i in range(6):
            chain_idx = i + 2
            if chain_idx < len(self.chain.links):
                val = self.current_joints[i]
                bounds = self.chain.links[chain_idx].bounds
                if bounds:
                    lower, upper = bounds
                    if lower is not None: val = max(lower, val)
                    if upper is not None: val = min(upper, val)
                ik_seed[chain_idx] = val

        # Get current Cartesian position for interpolation
        current_fk = self.chain.forward_kinematics(ik_seed)
        current_pos = current_fk[:3, 3]
        
        dist = np.linalg.norm(target_pos - current_pos)
        
        # If we are very close, do nothing
        if dist < 0.0025:
            return

        # Interpolate path to create straight line motion
        # Resolution: 1cm per point
        step_size = 0.05
        num_points = int(np.ceil(dist / step_size))
        # if num_points < 5: num_points = 5 # Minimum points for smoothness
        
        total_duration = dist / velocity
        # Ensure reasonable minimum duration
        if total_duration < 0.1: total_duration = 0.2
        
        dt = total_duration / num_points

        # Construct and publish trajectory
        traj_msg = JointTrajectory()
        
        # Standard UR joint names. 
        traj_msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        for i in range(1, num_points + 1):
            alpha = i / num_points
            # Linear interpolation in Cartesian space
            p_interp = current_pos + (target_pos - current_pos) * alpha
            
            # Calculate Inverse Kinematics for this waypoint
            ik_solution = self.chain.inverse_kinematics(
                target_position=p_interp,
                target_orientation=target_orientation,
                orientation_mode="all",
                initial_position=ik_seed
            )
            
            # Update seed for next point to ensure continuity
            ik_seed = ik_solution
            
            point = JointTrajectoryPoint()
            point.positions = list(ik_solution[2:8])
            
            t = dt * i
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            
            traj_msg.points.append(point)
            
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"Published linear trajectory: {dist:.3f}m in {total_duration:.2f}s ({num_points} pts)")

def main(args=None):
    rclpy.init(args=args)
    node = URMoveToConfig()
    
    # Spin once to ensure the message is sent, or keep spinning if you want to listen for feedback
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
