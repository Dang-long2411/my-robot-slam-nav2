import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import csv

def extract_trajectory(bag_path, topic):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_dict = {t.name: t.type for t in topic_types}
    traj = []

    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            traj.append([p.x, p.y])

    return np.array(traj)

def save_csv(traj, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])
        for x, y in traj:
            writer.writerow([x, y])
    print(f"[✔] Saved trajectory to {filename}")

def plot_trajectory(traj, label='SLAM Trajectory', color='blue'):
    x, y = traj[:, 0], traj[:, 1]
    plt.plot(x, y, label=label, color=color)

# === CONFIG ===
bag_path = '/home/hoanglong/myrobot_ws/slam_run15'
slam_topic = '/diff_cont/odom'
csv_output = 'slam_traj.csv'

# === RUN ===
rclpy.init()
slam_traj = extract_trajectory(bag_path, slam_topic)
rclpy.shutdown()

if len(slam_traj) == 0:
    print("[❌] No data found in the topic.")
else:
    save_csv(slam_traj, csv_output)

    plt.figure(figsize=(8, 8))
    plot_trajectory(slam_traj)
    plt.title('SLAM Odometry Trajectory')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()
