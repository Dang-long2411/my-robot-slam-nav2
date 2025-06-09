import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np
import matplotlib.pyplot as plt
import transforms3d
from scipy.interpolate import interp1d
import csv

# ---------------------------- TRANSFORM TO MATRIX ---------------------------- #
def transform_to_matrix(t):
    trans = np.eye(4)
    trans[0, 3] = t.translation.x
    trans[1, 3] = t.translation.y
    trans[2, 3] = t.translation.z
    quat = [t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z]
    rot = transforms3d.quaternions.quat2mat(quat)
    trans[0:3, 0:3] = rot
    return trans

# ---------------------------- EXTRACT SLAM ---------------------------------- #
def extract_trajectory_with_time(bag_path, topic):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    traj = []
    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            time_sec = t * 1e-9
            traj.append([time_sec, p.x, p.y])
    return np.array(traj)

# ---------------------------- EXTRACT GROUND TRUTH -------------------------- #
def extract_composed_tf_with_time(bag_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    latest_map_to_odom = None
    latest_odom_to_base = None
    trajectory = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != '/tf':
            continue
        msg = deserialize_message(data, TFMessage)
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if parent == 'map' and child == 'odom':
                latest_map_to_odom = transform_to_matrix(transform.transform)
            elif parent == 'odom' and child == 'base_link':
                latest_odom_to_base = transform_to_matrix(transform.transform)

        if latest_map_to_odom is not None and latest_odom_to_base is not None:
            composed = np.dot(latest_map_to_odom, latest_odom_to_base)
            x = composed[0, 3]
            y = composed[1, 3]
            time_sec = t * 1e-9
            trajectory.append([time_sec, x, y])
    return np.array(trajectory)

# ---------------------------- RMSE ----------------------------------------- #
def compute_rmse_time_aligned(gt, slam):
    if len(gt) < 2 or len(slam) < 2:
        return None, None

    gt_time = gt[:, 0]
    slam_time = slam[:, 0]

    interp_x = interp1d(slam_time, slam[:, 1], fill_value="extrapolate")
    interp_y = interp1d(slam_time, slam[:, 2], fill_value="extrapolate")
    slam_interp = np.vstack((interp_x(gt_time), interp_y(gt_time))).T

    error = np.linalg.norm(gt[:, 1:3] - slam_interp, axis=1)
    return np.sqrt(np.mean(error ** 2)), slam_interp

# ---------------------------- SAVE ----------------------------------------- #
def save_csv(traj, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'x', 'y'])
        for row in traj:
            writer.writerow(row)
    print(f"[✔] Saved to {filename}")

# ---------------------------- PLOT ----------------------------------------- #
def plot(gt, slam, rmse):
    plt.figure(figsize=(8, 8))
    plt.plot(gt[:, 1], gt[:, 2], 'r-', label='Ground Truth')
    plt.plot(slam[:, 0], slam[:, 1], 'b-', label='SLAM (Interpolated)')
    plt.title(f'SLAM vs Ground Truth\nRMSE = {rmse:.4f} m')
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()

# ---------------------------- MAIN ------------------------------------------ #
if __name__ == '__main__':
    bag_path = '/home/hoanglong/myrobot_ws/slam_run7'  # <-- CHANGE if needed
    slam_topic = '/diff_cont/odom'

    rclpy.init()
    slam_traj = extract_trajectory_with_time(bag_path, slam_topic)
    gt_traj = extract_composed_tf_with_time(bag_path)
    rclpy.shutdown()

    if len(slam_traj) == 0:
        print("[❌] No SLAM data found.")
    else:
        save_csv(slam_traj, 'slam.csv')

    if len(gt_traj) == 0:
        print("[❌] No Ground Truth data found.")
    else:
        save_csv(gt_traj, 'ground_truth.csv')

    if len(slam_traj) > 0 and len(gt_traj) > 0:
        rmse, slam_interp = compute_rmse_time_aligned(gt_traj, slam_traj)
        print(f"\n✅ Accurate Time-Aligned RMSE: {rmse:.4f} m")
        plot(gt_traj, slam_interp, rmse)
    else:
        print("\n⚠️ Could not compute RMSE.")
