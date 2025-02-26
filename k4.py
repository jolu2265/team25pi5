import numpy as np

class KalmanFilter:
    def __init__(self, F, H, Q, R, x0, P0):
        self.F = F  # State transition model
        self.H = H  # Measurement model
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = x0  # State estimate
        self.P = P0  # Covariance matrix

    def predict(self):
        """Predict the next state"""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        """Update with measurement"""
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman Gain
        y = z - np.dot(self.H, self.x)  # Measurement residual
        self.x = self.x + np.dot(K, y)
        
        I = np.eye(self.P.shape[0])
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        return self.x

# ----------------------------------------
# SYSTEM MODEL SETUP
# ----------------------------------------

dt = 1  # Time step

# State transition model (position updates via velocity)
F = np.array([[1, 0, dt, 0],  
              [0, 1, 0, dt],  
              [0, 0, 1, 0],  
              [0, 0, 0, 1]])  

# Measurement matrix (measuring distance + velocity)
H = np.array([[1, 1, 0, 0],  
              [0, 0, 1, 0],  
              [0, 0, 0, 1]])  

Q = np.diag([1e-3, 1e-3, 1e-2, 1e-2])  # Process noise
R = np.diag([0.1, 0.1, 0.1])  # Measurement noise

# Initial state (assume starting at (0,0), velocity (0,0))
x0 = np.array([[0], [0], [0], [0]])  
P0 = np.eye(4)  # Initial uncertainty

kf = KalmanFilter(F, H, Q, R, x0, P0)

# ----------------------------------------
# PD CONTROLLER + THRUSTER LOGIC
# ----------------------------------------

def pd_controller(kf_state, target_x=0, target_y=0, Kp=1.0, Kd=0.5):
    """PD control to generate thrust commands"""
    x, y, vx, vy = kf_state.flatten()

    # Compute control force
    Fx = Kp * (target_x - x) + Kd * (0 - vx)
    Fy = Kp * (target_y - y) + Kd * (0 - vy)

    return Fx, Fy

def thruster_control(Fx, Fy, threshold=1.0):
    """Convert force commands into binary thruster activations"""
    thrusters = {'left': 0, 'right': 0, 'up': 0, 'down': 0}

    if Fx > threshold:
        thrusters['right'] = 1
    elif Fx < -threshold:
        thrusters['left'] = 1
    
    if Fy > threshold:
        thrusters['up'] = 1
    elif Fy < -threshold:
        thrusters['down'] = 1

    return thrusters

# ----------------------------------------
# TEST SIMULATION LOOP
# ----------------------------------------

measurements = [
    np.array([[10], [2], [1]]),  
    np.array([[9], [1.5], [0.5]]),  
    np.array([[8.5], [1], [0]]),  
    np.array([[7], [0.5], [-0.5]]),
    np.array([[6], [0], [-1]]),  
]

for i, z in enumerate(measurements):
    print(f"\n--- Time step {i+1} ---")

    # Predict state
    kf.predict()

    # Update with measurement
    kf.update(z)
    
    # Get updated state
    state = kf.x
    print("Updated state:\n", state.flatten())

    # Compute PD control forces
    Fx, Fy = pd_controller(state)
    print(f"PD Controller Output - Fx: {Fx}, Fy: {Fy}")

    # Compute thruster commands
    thrusters = thruster_control(Fx, Fy)
    print("Thruster Commands:", thrusters)
