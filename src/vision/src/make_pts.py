import numpy as np

DEG2RAD = 0.0174532925 # (np.pi / 180)

# deg (pitch, yaw, fov)
extrinsic_angle = np.array([10, 0, 78], dtype=np.float32)
extrinsic_angle *= DEG2RAD

pitch, yaw, fov = extrinsic_angle
alpha = fov / 2

# m (ldh)
extrinsic_pos = np.array([0, 0, 1], dtype=np.float32)
l, d, h = extrinsic_pos

d_long = h / np.tan(pitch)
d_short = h / np.tan(pitch + alpha)

center_x_far = d_long * np.cos(yaw)
center_y_far = d_long * np.sin(yaw)

center_x_near = d_short * np.cos(yaw)
center_y_near = d_short * np.sin(yaw)

k_long = d_long / np.cos(alpha)
k_short = d_short / np.cos(alpha)

pA = np.array([np.cos(alpha + yaw), np.sin(alpha + yaw)]) * k_long * 100
pB = np.array([np.cos(yaw - alpha), np.sin(yaw - alpha)]) * k_long * 100

pC = np.array([np.cos(alpha + yaw), np.sin(alpha + yaw)]) * k_short * 100
pD = np.array([np.cos(yaw - alpha), np.sin(yaw - alpha)]) * k_short * 100

long_side = np.linalg.norm(pA - pB)
short_side = np.linalg.norm(pC - pD)

# lt, lb, rb, rt
top_y_pos = (d_long - d_short) * 100
pts = np.array([[0., 0.],
                [long_side, 0.],
                [(long_side - short_side) / 2, top_y_pos],
                [(long_side + short_side) / 2, top_y_pos]], dtype=np.int32)

print(pts)
