import os
import shutil
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from geometry_msgs.msg import TwistStamped

# === CONFIG ===
input_bag = '/home/hoanglong/myrobot_ws/slam_run_0.5'
output_bag = '/home/hoanglong/myrobot_ws/slam_run_0.1'
scale = 0.1 / 0.5  # scale factor for linear velocity

# === Auto-delete old output bag if exists ===
if os.path.exists(output_bag):
    print(f"[⚠] Deleting existing output bag: {output_bag}")
    shutil.rmtree(output_bag)

# === Open reader ===
reader = SequentialReader()
reader.open(
    StorageOptions(uri=input_bag, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

# === Setup writer ===
writer = SequentialWriter()
writer.open(
    StorageOptions(uri=output_bag, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

# Register the same topics
for topic_info in reader.get_all_topics_and_types():
    writer.create_topic(topic_info)

# === Rewrite messages with scaled velocity ===
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/diff_cont/cmd_vel_unstamped':
        msg = deserialize_message(data, TwistStamped)
        msg.twist.linear.x *= scale
        data = serialize_message(msg)
    writer.write(topic, data, t)

print(f"[✔] Scaled velocity bag saved to {output_bag}")
