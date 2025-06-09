import matplotlib.pyplot as plt

# Velocity values (from 0.25 m/s to 0.7 m/s in 0.05 steps)
velocities = [0.25 + 0.05 * i for i in range(10)]

# Corresponding Precision values (%)
precision_values = [
    97.4950, 94.3285, 91.9355, 86.7149, 82.2397,
    80.7978, 77.4921, 75.1346, 70.8316, 63.3558
]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(velocities, precision_values, marker='o', linestyle='-', color='green', label='Precision')

# Add titles and labels
plt.title('', fontsize=14)
plt.xlabel('Velocity (m/s)', fontsize=12)
plt.ylabel('Precision (%)', fontsize=12)
plt.xlim(0.25, 0.7)
plt.ylim(60, 100)
plt.legend()
plt.tight_layout()

# Save the plot (optional)
plt.savefig("velocity_vs_precision.png", dpi=300)

# Display the plot
plt.show()