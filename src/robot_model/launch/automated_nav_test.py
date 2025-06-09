import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
import csv
import math
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class AutomatedNavTest(Node):
    def __init__(self):
        super().__init__('automated_nav_test')
        self.navigator = BasicNavigator()

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return float('nan'), float('nan')

def main():
    rclpy.init()
    tester = AutomatedNavTest()
    navigator = tester.navigator

    navigator.lifecycleStartup()
    print("[INFO] Waiting for Nav2 to become active...")

    print("[INFO] Nav2 is active. Starting the navigation test...")

    goals = [
        (0.0, 1.0, 0.0),
        (3.0, -2.0, 0.0),
        (1.0, 2.0, 3.14),
        (-1.0, 2.0, -1.57),
        (-3.5, 0.0, 0.0)
    ]

    results = []

    for idx, (x, y, yaw) in enumerate(goals):
        print(f"[INFO] Sending goal {idx + 1}: ({x}, {y}, {yaw})")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        navigator.goToPose(goal_pose)

        start_time = time.time()
        while not navigator.isTaskComplete():
            rclpy.spin_once(tester, timeout_sec=0.1)
        end_time = time.time()

        result = navigator.getResult()

        # Get actual pose AFTER navigation completed
        actual_x, actual_y = tester.get_current_pose()
        error = euclidean_distance(x, y, actual_x, actual_y)

        if result == TaskResult.SUCCEEDED:
            print(f"[✔] Goal {idx + 1} reached. Error = {error:.3f} m.")
            success = "Success"
        else:
            print(f"[❌] Goal {idx + 1} failed. Error = {error:.3f} m.")
            success = "Failed"

        results.append([
            idx + 1,
            x, y,
            actual_x, actual_y,
            error,
            success,
            end_time - start_time
        ])

    # === Save to CSV ===
    csv_filename = "navigation_test_results.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Goal Number', 'Expected X', 'Expected Y', 'Actual X', 'Actual Y', 'Error (m)', 'Result', 'Time (s)'])
        writer.writerows(results)

    print(f"[📄] Navigation results saved to {csv_filename}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
