import numpy as np

biasx = -1.27

square = 0.48

width = 9*square
height = 3*square
radius = 2*square

spacing_len = 0.05
spacing_rad = spacing_len/square

theta = -0.00841866005

# Lines
l1x = np.arange(radius, radius + width, spacing_len, dtype=np.float)
l1y = np.zeros_like(l1x, dtype=np.float)
l2x = np.arange(radius + width, radius, -spacing_len, dtype=np.float)
l2y = np.ones_like(l1x, dtype=np.float)*7*square

l1 = np.stack([l1x + biasx, l1y], axis=0).T
l2 = np.stack([l2x + biasx, l2y], axis=0).T

l3y = np.arange(radius + height, radius, -spacing_len, dtype=np.float)
l3x = np.zeros_like(l3y, dtype=np.float)
l4y = np.arange(radius, radius + height, spacing_len, dtype=np.float)
l4x = np.ones_like(l4y, dtype=np.float)*13*square

l3 = np.stack([l3x + biasx, l3y], axis=0).T
l4 = np.stack([l4x + biasx, l4y], axis=0).T

# Quads
theta1 = np.arange(np.pi, 3*np.pi/2, spacing_rad, dtype=np.float)
theta2 = np.arange(3*np.pi/2, 2*np.pi, spacing_rad, dtype=np.float)
theta3 = np.arange(0, np.pi/2, spacing_rad, dtype=np.float)
theta4 = np.arange(np.pi/2, np.pi, spacing_rad, dtype=np.float)

r1x = radius*np.cos(theta1) + radius
r1y = radius*np.sin(theta1) + radius
r2x = radius*np.cos(theta2) + radius + width
r2y = radius*np.sin(theta2) + radius
r3x = radius*np.cos(theta3) + radius + width
r3y = radius*np.sin(theta3) + radius + height
r4x = radius*np.cos(theta4) + radius
r4y = radius*np.sin(theta4) + radius + height

r1 = np.stack([r1x + biasx, r1y], axis=0).T
r2 = np.stack([r2x + biasx, r2y], axis=0).T
r3 = np.stack([r3x + biasx, r3y], axis=0).T
r4 = np.stack([r4x + biasx, r4y], axis=0).T

l = np.concatenate([l1, r2, l4, r3, l2, r4, l3, r1], axis=0)
Rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]).T

l = np.dot(l, Rot)

np.savetxt('path.csv', l, fmt='%.18e', delimiter=',')