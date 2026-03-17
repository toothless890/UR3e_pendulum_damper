import numpy as np

class AccelerationKalmanFilter:
    """
    Kalman Filter designed to estimate linear and angular motion parameters.
    
    State vector: [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]^T
    
    Estimates:
    - Linear position, velocity, and acceleration
    - Angular position, angular velocity, and angular acceleration
    
    Assumes constant-acceleration model for both linear and angular motion.
    
    Parameters
    ----------
    dt : float
        Time step between measurements (seconds)
    process_noise_std : float or array
        Standard deviation of process noise. Can be:
        - Single float: applied equally to all states
        - Array [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]: individual per state
    measurement_noise_std : float or tuple
        Standard deviation of measurement noise. Can be:
        - Single float: applied to both linear and angular position measurements
        - Tuple (lin_noise, ang_noise): separate noise for position types
    initial_state : array, shape (6,), optional
        Initial estimate [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
    initial_covariance : float
        Initial estimate uncertainty (diagonal value for P matrix)
    """
    
    def __init__(self, dt, process_noise_std=0.1, measurement_noise_std=0.5,
        initial_state=None, initial_covariance=1.0):
        self.dt = dt
        
        # State transition matrix (block diagonal kinematic equations)
        # Linear motion: pos[k+1] = pos[k] + vel[k]*dt + 0.5*acc[k]*dt^2
        #               vel[k+1] = vel[k] + acc[k]*dt
        #               acc[k+1] = acc[k]
        # Angular motion: ang_pos[k+1] = ang_pos[k] + ang_vel[k]*dt + 0.5*ang_acc[k]*dt^2
        #                 ang_vel[k+1] = ang_vel[k] + ang_acc[k]*dt
        #                 ang_acc[k+1] = ang_acc[k]
        
        dt2 = dt**2
        F_block = np.array([
            [1, dt, 0.5*dt2],
            [0, 1,   dt],
            [0, 0,   1]
        ])
        
        self.F = np.zeros((6, 6))
        self.F[:3, :3] = F_block      # Linear block
        self.F[3:6, 3:6] = F_block    # Angular block
        
        # Measurement matrix: measure linear position and angular position
        # z = [lin_pos, ang_pos]
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        
        # Process noise covariance (Q)
        if isinstance(process_noise_std, (list, np.ndarray)):
            q_ = np.array(process_noise_std)**2
        else:
            # Default: noise decreases for velocity and acceleration estimates
            q_ = np.array([process_noise_std, process_noise_std*0.5, process_noise_std*0.1,
                          process_noise_std, process_noise_std*0.5, process_noise_std*0.1])**2
        
        self.Q = np.diag(q_)
        
        # Measurement noise covariance (R)
        if isinstance(measurement_noise_std, (tuple, list)):
            lin_meas_noise, ang_meas_noise = measurement_noise_std
            self.R = np.diag([lin_meas_noise**2, ang_meas_noise**2])
        else:
            self.R = np.diag([measurement_noise_std**2, measurement_noise_std**2])
        
        # State estimate [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
        if initial_state is None:
            self.x = np.zeros((6, 1))
        else:
            self.x = np.array(initial_state).reshape(6, 1)
        
        # Covariance estimate (uncertainty in state estimate)
        self.P = np.eye(6) * initial_covariance
        
    def predict(self, u=None):
        """
        Prediction step: predict next state given optional control input.
        
        Parameters
        ----------
        u : float or array, optional
            Control input (not used in constant-acceleration model, reserved for future).
        
        Returns
        -------
        state : ndarray, shape (6,)
            Predicted [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.flatten()
    
    def update(self, z_lin_pos, z_ang_pos):
        """
        Update step: update state estimate based on linear and angular position measurements.
        
        Parameters
        ----------
        z_lin_pos : float
            Measured linear position
        z_ang_pos : float
            Measured angular position
        
        Returns
        -------
        state : ndarray, shape (6,)
            Updated [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
        """
        z = np.array([[z_lin_pos], [z_ang_pos]])
        y = z - self.H @ self.x
        
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x.flatten()
    
    # Linear motion accessors
    def get_linear_position(self):
        """Return current linear position estimate."""
        return self.x[0, 0]
    
    def get_linear_velocity(self):
        """Return current linear velocity estimate."""
        return self.x[1, 0]
    
    def get_linear_acceleration(self):
        """Return current linear acceleration estimate."""
        return self.x[2, 0]
    
    # Angular motion accessors
    def get_angular_position(self):
        """Return current angular position estimate (radians)."""
        return self.x[3, 0]
    
    def get_angular_velocity(self):
        """Return current angular velocity estimate (rad/s)."""
        return self.x[4, 0]
    
    def get_angular_acceleration(self):
        """Return current angular acceleration estimate (rad/s²)."""
        return self.x[5, 0]
    
    # Convenience aliases
    def get_position(self):
        """Return current linear position estimate."""
        return self.get_linear_position()
    
    def get_velocity(self):
        """Return current linear velocity estimate."""
        return self.get_linear_velocity()
    
    def get_acceleration(self):
        """Return current linear acceleration estimate."""
        return self.get_linear_acceleration()
    
    def get_state(self):
        """Return full state vector [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]."""
        return self.x.flatten()


# Legacy interface (kept for compatibility)
class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0

    def predict(self, u):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q
        return self.x
    
    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        return self.x


if __name__ == "__main__":
    # Example: Tracking a robot with linear and angular motion
    dt = 0.1  # 100 Hz sampling rate
    
    # Create filter tuned for both linear and angular motion estimation
    kf = AccelerationKalmanFilter(
        dt=dt,
        # Process noise: [lin_pos, lin_vel, lin_acc, ang_pos, ang_vel, ang_acc]
        process_noise_std=[0.01, 0.05, 0.1, 0.005, 0.05, 0.1],
        # Measurement noise: (linear position noise, angular position noise)
        measurement_noise_std=(0.5, 0.1),
        initial_state=[0, 0, 0, 0, 0, 0],
        initial_covariance=1.0
    )
    
    # Simulate robot motion: constant linear and angular acceleration
    true_lin_acc = 1.0        # m/s^2
    true_ang_acc = 0.5        # rad/s^2
    true_lin_pos = 0
    true_lin_vel = 0
    true_ang_pos = 0
    true_ang_vel = 0
    
    print("Time(s) | Est Lin Pos | Est Lin Acc | Est Ang Pos | Est Ang Acc | True Lin Pos | True Ang Pos")
    print("-" * 100)
    
    for step in range(50):
        # Update true motion
        true_lin_vel += true_lin_acc * dt
        true_lin_pos += true_lin_vel * dt + 0.5 * true_lin_acc * dt**2
        
        true_ang_vel += true_ang_acc * dt
        true_ang_pos += true_ang_vel * dt + 0.5 * true_ang_acc * dt**2
        
        # Add noise to simulated measurements
        lin_meas_noise = np.random.normal(0, 0.5)
        ang_meas_noise = np.random.normal(0, 0.1)
        measured_lin_pos = true_lin_pos + lin_meas_noise
        measured_ang_pos = true_ang_pos + ang_meas_noise
        
        # Filter: predict then update
        kf.predict()
        kf.update(measured_lin_pos, measured_ang_pos)
        
        # Print results every 5 steps
        if step % 5 == 0:
            t = step * dt
            print(f"{t:6.1f}  | {kf.get_linear_position():11.3f} | {kf.get_linear_acceleration():11.3f} | "
                  f"{kf.get_angular_position():11.3f} | {kf.get_angular_acceleration():11.3f} | "
                  f"{true_lin_pos:12.3f} | {true_ang_pos:12.3f}")
    
    print("\nFinal state:")
    print(f"  Linear position:      {kf.get_linear_position():.4f} m")
    print(f"  Linear velocity:      {kf.get_linear_velocity():.4f} m/s")
    print(f"  Linear acceleration:  {kf.get_linear_acceleration():.4f} m/s²")
    print(f"  Angular position:     {kf.get_angular_position():.4f} rad")
    print(f"  Angular velocity:     {kf.get_angular_velocity():.4f} rad/s")
    print(f"  Angular acceleration: {kf.get_angular_acceleration():.4f} rad/s²")