import pandas as pd

# Define evaluation metrics for Nav2 performance testing
nav2_test_criteria = {
    "Test Category": [
        "Global Planner",
        "Local Planner (Controller)",
        "SLAM Integration",
        "Real-Time Performance",
        "Obstacle Avoidance",
        "Localization Accuracy",
        "Recovery Behaviors",
        "Navigation Success Rate",
        "Multi-run Consistency",
        "Parameter Sensitivity",
    ],
    "Test Description": [
        "Assess if global planner finds optimal paths in various map topologies",
        "Evaluate smoothness and safety of local trajectory execution",
        "Verify map quality and pose estimation accuracy with SLAM Toolbox",
        "Measure latency and system responsiveness during navigation",
        "Test static and dynamic obstacle avoidance effectiveness",
        "Evaluate drift and localization stability over time",
        "Observe robot's response to goal failures or being stuck",
        "Record rate of successfully reached goals vs. failures",
        "Compare repeatability of paths and timings across runs",
        "Assess how tuning velocity, inflation, tolerance affects navigation"
    ],
    "Metric/Notes": [
        "Path length, deviation from shortest path",
        "Trajectory curvature, jerkiness, safe distances",
        "RMSE, loop closure effectiveness, map completeness",
        "Time-to-goal, lag in replanning, CPU usage",
        "Minimum safe distance maintained, response time",
        "RMSE, visual offset in RViz, pose error drift",
        "Behavior triggered, success of recovery attempts",
        "Success %, average time and distance per goal",
        "RMSE between runs, path overlap %",
        "RMSE or goal success before/after param change"
    ]
}

# Create DataFrame
df_nav2_test_plan = pd.DataFrame(nav2_test_criteria)
df_nav2_test_plan
