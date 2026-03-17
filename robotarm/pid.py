import math
from simple_pid import PID
from serial_monitor import getData

class InversePendulumPID():
    def __init__(self, dt=0.01):
        # Time step (seconds)
        self.dt = dt
        
        # Angular state variables
        self.measured_ang_acceleration = 0.0
        self.predicted_ang_acceleration = 0.0
        self.measured_angle = 0.0
        self.predicted_angle = 0.0
        self.measured_ang_velocity = 0.0
        self.predicted_ang_velocity = 0.0
        
        # Pendulum parameters
        self.length = 0.5  # meters
        
        # Linear state variables
        self.measured_linear_acceleration = 0.0
        self.predicted_linear_acceleration = 0.0
        self.corrected_linear_acceleration = 0.0
        
        # Kalman filter for state estimation
        # Tuned for pendulum: [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
        self.filter = sample_kalman.AccelerationKalmanFilter(
            dt=dt,
            process_noise_std=[0.01, 0.05, 0.1, 0.01, 0.05, 0.1],
            measurement_noise_std=(0.1, 0.05),  # (linear pos noise, angular pos noise)
            initial_state=[0, 0, 0, 0, 0, 0],
            initial_covariance=1.0
        )
        
        # PID controller to normalize acceleration to 0
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0.0)
        self.pid.sample_time = dt

    def control_loop(self, measured_angle: float, measured_ang_velocity: float, 
                     measured_ang_acceleration: float, measured_linear_position: float = 0.0):
        """
        Main control loop that uses Kalman filter to predict pendulum state.
        
        Parameters
        ----------
        measured_angle : float
            Measured angular position (radians)
        measured_ang_velocity : float
            Measured angular velocity (rad/s)
        measured_ang_acceleration : float
            Measured angular acceleration (rad/s²)
        measured_linear_position : float, optional
            Measured linear position (meters), default 0.0
        """
        # Store measured angular states
        self.measured_angle = measured_angle
        self.measured_ang_velocity = measured_ang_velocity
        self.measured_ang_acceleration = measured_ang_acceleration
        
        # Calculate measured linear acceleration from pendulum dynamics
        # a_linear = (L * α) / cos(θ) - g * tan(θ)
        try:
            cos_angle = math.cos(measured_angle)
            tan_angle = math.tan(measured_angle)
            if abs(cos_angle) < 0.01:  # Avoid division by near-zero
                self.measured_linear_acceleration = 0.0
            else:
                self.measured_linear_acceleration = (
                    (self.length * measured_ang_acceleration) / cos_angle 
                    - 9.8 * tan_angle
                )
        except:
            self.measured_linear_acceleration = 0.0
        
        # Kalman filter: predict next state
        self.filter.predict()
        
        # Update filter with current measurements
        # The filter expects linear and angular position measurements
        self.filter.update(measured_linear_position, measured_angle)
        
        # Extract predicted states from filter
        self.predicted_linear_acceleration = self.filter.get_linear_acceleration()
        self.predicted_angle = self.filter.get_angular_position()
        self.predicted_ang_velocity = self.filter.get_angular_velocity()
        self.predicted_ang_acceleration = self.filter.get_angular_acceleration()
        
        # Use predicted linear acceleration with PID controller
        # Get corrective control signal from PID to drive acceleration to 0
        corrective_signal = self.pid(self.predicted_linear_acceleration)
        
        # Output the counteracting force in opposite direction
        self.corrected_linear_acceleration = -corrective_signal
        
        # Print component info
        print(f"Measured angle: {measured_angle:.4f} rad, ang_vel: {measured_ang_velocity:.4f} rad/s")
        print(f"Measured linear accel: {self.measured_linear_acceleration:.6f} m/s²")
        print(f"Predicted linear accel: {self.predicted_linear_acceleration:.6f} m/s²")
        print(f"  P (proportional): {self.pid._proportional:.6f}")
        print(f"  I (integral): {self.pid._integral:.6f}")
        print(f"  D (derivative): {self.pid._derivative:.6f}")
        print(f"Corrected acceleration: {self.corrected_linear_acceleration:.6f}")
        return self.corrected_linear_acceleration

def reset(self):
    self.measured_linear_acceleration = 0.0
    self.predicted_linear_acceleration = 0.0
    self.corrected_linear_acceleration = 0.0
    self.pid.reset()

def main():
    """Demo: Inverse pendulum control using Kalman filter state estimation."""
    dt = 0.01  # 100 Hz control loop
    controller = InversePendulumPID(dt=dt)
    
    print("Inverse Pendulum PID Control with Kalman Filter State Estimation\n")
    print("Testing with different angular accelerations:\n")
    
    # Test cases: [angle, ang_velocity, ang_acceleration]
    test_cases = [
        (0.1, 0.5, -2.5, "Small angle with negative angular accel"),
        (0.0, 0.0, 0.0, "Balanced at zero angle"),
        (-0.1, -0.5, 2.5, "Negative angle with positive angular accel"),
    ]
    
    for idx, (angle, ang_vel, ang_acc, description) in enumerate(test_cases, start=1):
        reset(controller)
        print(f"Test {idx}: {description}")
        print(f"  Input: angle={angle:.2f} rad, ang_vel={ang_vel:.2f} rad/s, ang_acc={ang_acc:.2f} rad/s²")
        output = controller.control_loop(
            measured_angle=angle,
            measured_ang_velocity=ang_vel,
            measured_ang_acceleration=ang_acc,
            measured_linear_position=0.0
        )
        print(f"  Output: corrected_accel={output:.3f} m/s²\n")


if __name__ == "__main__":
    main()
        
    