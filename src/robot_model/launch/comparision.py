import cv2
import matplotlib.pyplot as plt
import yaml
import numpy as np
import matplotlib.patches as mpatches

def load_map(pgm_path, yaml_path):
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)
    resolution = metadata['resolution']
    origin = metadata['origin']
    return img, resolution, origin

def calculate_accuracy(map1, map2):
    # Convert to binary: occupied (1) if pixel < 128, else free (0)
    binary1 = (map1 < 128).astype(np.uint8)
    binary2 = (map2 < 128).astype(np.uint8)

    TP = np.sum((binary1 == 1) & (binary2 == 1))
    TN = np.sum((binary1 == 0) & (binary2 == 0))
    FP = np.sum((binary1 == 1) & (binary2 == 0))
    FN = np.sum((binary1 == 0) & (binary2 == 1))

    total = TP + TN + FP + FN
    accuracy = (TP + TN) / total if total != 0 else 0
    precision = TP / (TP + FP) if (TP + FP) != 0 else 0
    recall = TP / (TP + FN) if (TP + FN) != 0 else 0
    f1_score = (2 * precision * recall) / (precision + recall) if (precision + recall) != 0 else 0

    return {
        'Accuracy': accuracy * 100,
        'Precision': precision * 100,
        'Recall': recall * 100,
        'F1 Score': f1_score * 100,
        'TP': TP,
        'FP': FP,
        'TN': TN,
        'FN': FN
    }

def overlay_maps(map1, res1, origin1, map2, res2, origin2):
    scale_factor = res1 / res2
    map2_rescaled = cv2.resize(map2, (0, 0), fx=scale_factor, fy=scale_factor)

    offset_x = int((origin2[0] - origin1[0]) / res1)
    offset_y = int((origin2[1] - origin1[1]) / res1)

    h1, w1 = map1.shape
    h2, w2 = map2_rescaled.shape
    h = max(h1 + abs(offset_y), h2 + abs(offset_y))
    w = max(w1 + abs(offset_x), w2 + abs(offset_x))

    canvas1 = np.full((h, w), 255, dtype=np.uint8)
    canvas2 = np.full((h, w), 255, dtype=np.uint8)

    canvas1[max(0, offset_y):max(0, offset_y) + h1, max(0, offset_x):max(0, offset_x) + w1] = map1
    canvas2[:h2, :w2] = map2_rescaled

    # Threshold to get occupied areas
    mask1 = canvas1 < 128  # SLAM occupied
    mask2 = canvas2 < 128  # Ground Truth occupied

    # Create white background
    overlay = np.full((h, w, 3), 255, dtype=np.uint8)

    # Color SLAM map in blue
    overlay[mask1] = [0, 0, 255]

    # Color GT map in red
    overlay[mask2] = [255, 0, 0]

    # Overlap both = purple
    overlay[np.logical_and(mask1, mask2)] = [0, 0, 255]  # Purple

    return overlay


# === File Paths (replace with your paths) ===
slam_pgm = '/home/hoanglong/myrobot_ws/src/robot_model/worlds/map1_test7.pgm'
slam_yaml = '/home/hoanglong/myrobot_ws/src/robot_model/worlds/map1_test7.yaml'
gt_pgm = '/home/hoanglong/myrobot_ws/src/robot_model/worlds/map1_test1.pgm'
gt_yaml = '/home/hoanglong/myrobot_ws/src/robot_model/worlds/map1_test1.yaml'

# === Load and overlay maps ===
slam_map, slam_res, slam_origin = load_map(slam_pgm, slam_yaml)
gt_map, gt_res, gt_origin = load_map(gt_pgm, gt_yaml)

overlay = overlay_maps(slam_map, slam_res, slam_origin, gt_map, gt_res, gt_origin)

# Resize both maps to the same scale and origin
def prepare_aligned_binary_maps(map1, res1, origin1, map2, res2, origin2):
    scale_factor = res1 / res2
    map2_rescaled = cv2.resize(map2, (0, 0), fx=scale_factor, fy=scale_factor)

    offset_x = int((origin2[0] - origin1[0]) / res1)
    offset_y = int((origin2[1] - origin1[1]) / res1)

    h1, w1 = map1.shape
    h2, w2 = map2_rescaled.shape
    h = max(h1 + abs(offset_y), h2 + abs(offset_y))
    w = max(w1 + abs(offset_x), w2 + abs(offset_x))

    canvas1 = np.full((h, w), 255, dtype=np.uint8)
    canvas2 = np.full((h, w), 255, dtype=np.uint8)

    canvas1[max(0, offset_y):max(0, offset_y) + h1, max(0, offset_x):max(0, offset_x) + w1] = map1
    canvas2[:h2, :w2] = map2_rescaled

    return canvas1, canvas2

# Prepare aligned maps
aligned_slam_map, aligned_gt_map = prepare_aligned_binary_maps(slam_map, slam_res, slam_origin, gt_map, gt_res, gt_origin)

# Calculate accuracy metrics
metrics = calculate_accuracy(aligned_slam_map, aligned_gt_map)

# Print results
for key, value in metrics.items():
    print(f"{key}: {value:.4f}" if isinstance(value, float) else f"{key}: {value}")


# === Show the result with legend ===
plt.figure(figsize=(10, 10))
plt.imshow(overlay)
plt.title("Map 1")
plt.axis('off')

# Add legend
legend_elements = [
    mpatches.Patch(color='blue', label='SLAM Occupied'),
    mpatches.Patch(color='red', label='Ground Truth Occupied')
]
plt.legend(handles=legend_elements, loc='lower right')

plt.show()