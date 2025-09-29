import numpy as np
import time

# Simulation Parameters
max_dc = 100.0
pwm_freq = 100.0  # Hz
dt = 0.1  # Simulation timestep
sim_time = 30  # seconds


# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint = 0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# LQR Controller
def lqr_control(A, B, Q, R, x, x_ref):
    """
    Simple LQR control: u = -K*(x - x_ref)
    Solve continuous-time Algebraic Riccati Equation (CARE)
    """
    from scipy.linalg import solve_continuous_are
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    u = -K @ (x - x_ref)
    return u


# Initialize PID controllers for each valve
pid_controllers = [PID(1.0, 0.01, 0.1) for _ in range(5)]

# Simulation of pressures (for 5 valves)
pressures = np.zeros(5)
valve_dc = np.zeros(5)

# Example linear system for LQR (5 valves)
A = -0.5 * np.eye(5)  # simplified decay model
B = 0.5 * np.eye(5)
Q = np.eye(5)
R = 0.1 * np.eye(5)
x_ref = np.array([50, 50, 50, 50, 50])  # target pressures

# Main Loop
t = 0.0
while t < sim_time:
    # Update PID controllers for each valve
    for i in range(5):
        pid_controllers[i].setpoint = x_ref[i]
        valve_dc[i] = pid_controllers[i].update(pressures[i])
        valve_dc[i] = np.clip(valve_dc[i], 0, max_dc)

    # LQR correction
    lqr_u = lqr_control(A, B, Q, R, pressures, x_ref)
    valve_dc += lqr_u
    valve_dc = np.clip(valve_dc, 0, max_dc)

    # Update pressures (simulation of valve response)
    pressures += dt * (A @ pressures + B @ valve_dc)

    # Print for monitoring
    print(f"Time {t:.2f}s | Pressures: {pressures} | PWM: {valve_dc}")

    t += dt
    time.sleep(dt)
