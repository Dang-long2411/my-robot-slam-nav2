import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === Load Data ===
csv_file = '/home/hoanglong/myrobot_ws/src/robot_model/launch/navigation_test_results.csv'
data = pd.read_csv(csv_file)

# Clean column names (remove extra spaces if any)
data.columns = data.columns.str.strip()

print("[INFO] CSV loaded successfully.")
print(f"[INFO] CSV Columns: {data.columns.tolist()}")

# === Calculate Position Error (Euclidean Distance) ===
errors = []

for index, row in data.iterrows():
    expected_x = row['Expected X']
    expected_y = row['Expected Y']
    actual_x = row['Actual X']
    actual_y = row['Actual Y']
    
    if not np.isnan(actual_x) and not np.isnan(actual_y):
        error = np.sqrt((expected_x - actual_x)**2 + (expected_y - actual_y)**2)
    else:
        error = np.nan  # Handle missing data
    errors.append(error)

# Add error column
data['Position Error (m)'] = errors

# === Save updated data ===
output_csv = '/home/hoanglong/myrobot_ws/src/robot_model/launch/navigation_accuracy_results.csv'
data.to_csv(output_csv, index=False)
print(f"[✅] Updated results saved to: {output_csv}")

# === Summary Statistics ===
print("\n=== 📍 Navigation Position Accuracy Summary ===")
print(f"Average Position Error: {np.nanmean(errors):.4f} meters")
print(f"Max Position Error: {np.nanmax(errors):.4f} meters")
print(f"Min Position Error: {np.nanmin(errors):.4f} meters")

# === Plot Expected vs Actual positions ===
plt.figure(figsize=(8, 6))
plt.plot(data['Expected X'].to_numpy(), data['Expected Y'].to_numpy(), 'bo-', label='Expected Position')
plt.plot(data['Actual X'].to_numpy(), data['Actual Y'].to_numpy(), 'rx--', label='Actual Position')
plt.scatter(data['Expected X'].to_numpy(), data['Expected Y'].to_numpy(), c='blue', marker='o')
plt.scatter(data['Actual X'].to_numpy(), data['Actual Y'].to_numpy(), c='red', marker='x')
plt.title('Expected vs Actual Navigation Positions')
plt.xlabel('X position (meters)')
plt.ylabel('Y position (meters)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('/home/hoanglong/myrobot_ws/src/robot_model/launch/expected_vs_actual_plot.png')
plt.show()

print("[📈] Plot saved as 'expected_vs_actual_plot.png'.")
