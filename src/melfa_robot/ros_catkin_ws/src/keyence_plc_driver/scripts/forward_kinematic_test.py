#!/usr/bin/env python3
import numpy as np, math

def A0_user(t1):
    return np.array([
        [math.cos(t1), -math.sin(t1), 0.0, 0.0],
        [math.sin(t1),  math.cos(t1), 0.0, 0.0],
        [0.0,           0.0,           1.0, 0.0],
        [0.0,           0.0,           0.0, 1.0]
    ])

def A1_user(t2):
    return np.array([
        [math.cos(t2), 0.0, math.sin(t2), 0.0],
        [0.0,          1.0,  0.0,         0.0],
        [-math.sin(t2), 0.0,  math.cos(t2), 350.0],
        [0.0,          0.0,  0.0,         1.0]
    ])

def A2_user(t3):
    return np.array([
        [math.cos(t3), 0.0, math.sin(t3), 0.0],
        [0.0,          1.0,  0.0,          0.0],
        [-math.sin(t3), 0.0,  math.cos(t3), 310.0],
        [0.0,          0.0,  0.0,          1.0]
    ])

def A3_user(t4):
    return np.array([
        [math.cos(t4), -math.sin(t4), 0.0, -50.0],
        [math.sin(t4), math.cos(t4),  0.0, 0.0],
        [0.0,          0.0,           1.0, 50.0],
        [0.0,          0.0,           0.0, 1.0]
    ])

def A4_user(t5):
    return np.array([
        [math.cos(t5),  0.0, math.sin(t5), 0.0],
        [0.0,           1.0, 0.0,          0.0],
        [-math.sin(t5), 0.0, math.cos(t5), 285.15],
        [0.0,           0.0, 0.0,          1.0]
    ])


# angles you provided (I assume degrees)
angles_deg = [168.92, -57.16, 16.72, -96.2, 56.09]
angles = [math.radians(a) for a in angles_deg]

# Multiply all transformation matrices
T_user = (
    A0_user(angles[0]) @
    A1_user(angles[1]) @
    A2_user(angles[2]) @
    A3_user(angles[3]) @
    A4_user(angles[4]) 
)

pos_1x4 = T_user @ np.array([[0.0], [0.0], [124.85], [1.0]])

# Format and display results
np.set_printoptions(precision=6, suppress=True)

# Flatten the vector to make sure we can format the values
pos_values = pos_1x4[:3].flatten()

print("\n=== Homogeneous Transformation Matrix (T0_5) ===")
print(T_user)

print("\n=== End-Effector Position (1x4 vector, mm) ===")
print(pos_1x4.reshape(1, 4))






