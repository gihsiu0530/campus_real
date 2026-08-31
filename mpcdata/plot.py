import os

import matplotlib
matplotlib.use('Agg')  # headless: save figures without a display server
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Paths: reference path (ground truth) vs recorded odometry
base_dir = os.path.dirname(os.path.abspath(__file__))
repo_dir = os.path.dirname(base_dir)
ref_path = os.path.join(repo_dir, 'path', 'smoothed', 'back_garden_07new.csv')
odom_path = os.path.join(base_dir, 'bkgd0813', 'best.csv')
output_dir = os.path.join(base_dir, 'bkgd0813')

os.makedirs(output_dir, exist_ok=True)

# Reference path (ground truth)
df_ref = pd.read_csv(ref_path)
ref_xy = df_ref[['x', 'y']].to_numpy(dtype=float)

# Recorded odometry (exported from rosbag)
df_odom = pd.read_csv(odom_path)
odom_xy = df_odom[['field.pose.pose.position.x',
                   'field.pose.pose.position.y']].to_numpy(dtype=float)
odom_t = df_odom['%time'].to_numpy(dtype=float) / 1e9  # ns -> s

# Keep only the moving part of the run. The bag's twist field is all zeros, so
# speed is differentiated from the positions; a short rolling median rejects the
# position noise that would otherwise look like motion while parked.
SPEED_THRESHOLD = 0.1  # m/s
speed = np.hypot(np.gradient(odom_xy[:, 0], odom_t),
                 np.gradient(odom_xy[:, 1], odom_t))
speed = pd.Series(speed).rolling(5, center=True, min_periods=1).median().to_numpy()
moving = speed >= SPEED_THRESHOLD

print(f"Samples: {moving.sum()} moving / {len(moving)} total "
      f"({(~moving).sum()} stationary samples excluded)")

odom_xy = odom_xy[moving]


def signed_cross_track_error(points, path):
    """Signed distance from each point to the reference polyline.

    Every point is projected onto all path segments; the closest projection
    wins. The sign comes from the 2D cross product of the segment direction
    and the point offset (positive = left of the path).
    """
    seg_start = path[:-1]
    seg_vec = path[1:] - path[:-1]
    seg_len_sq = np.einsum('ij,ij->i', seg_vec, seg_vec)
    seg_len_sq[seg_len_sq == 0.0] = 1e-12

    # offset[i, j] = points[i] - seg_start[j]
    offset = points[:, None, :] - seg_start[None, :, :]

    # Clamp the projection parameter so it stays inside each segment
    t = np.einsum('ijk,jk->ij', offset, seg_vec) / seg_len_sq
    np.clip(t, 0.0, 1.0, out=t)

    foot = seg_start[None, :, :] + t[:, :, None] * seg_vec[None, :, :]
    diff = points[:, None, :] - foot
    dist = np.hypot(diff[:, :, 0], diff[:, :, 1])

    nearest = np.argmin(dist, axis=1)
    idx = np.arange(points.shape[0])
    cross = (seg_vec[nearest, 0] * offset[idx, nearest, 1]
             - seg_vec[nearest, 1] * offset[idx, nearest, 0])
    return np.sign(cross) * dist[idx, nearest]


cte = signed_cross_track_error(odom_xy, ref_xy)
abs_cte = np.abs(cte)

rmse_cte = np.sqrt(np.mean(cte ** 2))
mae_cte = abs_cte.mean()
max_cte = abs_cte.max()

print(f"CTE RMSE:      {rmse_cte:.4f} m")
print(f"CTE MAE:       {mae_cte:.4f} m")
print(f"CTE Max Error: {max_cte:.4f} m")

# Plot 1: CTE over time
plt.figure(figsize=(16, 6))
plt.plot(cte, label='cte')
plt.title('CTE (Cross Track Error)')
plt.xlabel('Time Steps')
plt.ylabel('cte (m)')
plt.xlim(0, len(cte) - 1)
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(output_dir, 'cte.png'), dpi=150)
plt.close()

# Plot 2: path comparison
plt.figure(figsize=(12, 10))
plt.plot(ref_xy[:, 0], ref_xy[:, 1],
         label='Reference Path (back_garden_07new)', color='black', linewidth=2)
plt.plot(odom_xy[:, 0], odom_xy[:, 1],
         label='Real Trajectory (odom)', color='red', linestyle='--')
plt.title('Path Comparison: Real Trajectory vs Reference Path')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(output_dir, 'path_comparison.png'), dpi=150)
plt.close()
