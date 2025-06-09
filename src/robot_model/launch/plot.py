import matplotlib.pyplot as plt
import numpy as np

# Set up figure
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-1, 7)
ax.set_ylim(-1, 5)

# Coordinate axes
ax.arrow(0, 0, 6, 0, head_width=0.1, length_includes_head=True)
ax.arrow(0, 0, 0, 4, head_width=0.1, length_includes_head=True)
ax.text(6.2, 0, 'X')
ax.text(0, 4.2, 'Y')
ax.text(0, -0.2, 'O')

# Center of circular motion
O = [1, 1]
car_center = [4.5, 1]
r = car_center[0] - O[0]

# Circle center
ax.plot(O[0], O[1], 'ko')
ax.text(O[0] - 0.2, O[1], 'O (Center of circular motion)')

# Car line
L = 1
left_wheel = [car_center[0] - L, car_center[1]]
right_wheel = [car_center[0] + L, car_center[1]]
ax.plot([left_wheel[0], right_wheel[0]], [left_wheel[1], right_wheel[1]], 'k', lw=2)
ax.plot(left_wheel[0], left_wheel[1], 'ko')
ax.plot(right_wheel[0], right_wheel[1], 'ko')

# Velocity arrows
ax.arrow(left_wheel[0], 0.5, 0, -0.3, head_width=0.1)
ax.arrow(right_wheel[0], 0.5, 0, -0.3, head_width=0.1)
ax.text(left_wheel[0] - 0.1, 0.1, 'vₗ')
ax.text(right_wheel[0] - 0.1, 0.1, 'vᵣ')

# Radius line
ax.plot([O[0], car_center[0]], [O[1], car_center[1]], 'k--')
ax.text((O[0]+car_center[0])/2, O[1] - 0.2, 'r')

# Angular velocity arc
theta = np.linspace(0, np.pi/6, 100)
arc = 0.4 * np.array([np.cos(theta), np.sin(theta)])
ax.plot(O[0] + arc[0], O[1] + arc[1], 'k')
ax.text(O[0]+0.5, O[1]+0.2, 'ω')

# Labels
ax.text(car_center[0] - L - 0.2, car_center[1] + 0.2, '$O_L$')
ax.text(car_center[0] + L + 0.1, car_center[1] + 0.2, '$O_{L1}$')
ax.text(car_center[0] - 0.2, car_center[1] - 0.2, 'Car body center')

plt.axis('off')
plt.show()