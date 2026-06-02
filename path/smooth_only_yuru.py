import pandas as pd
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('/home/cyc/campus_ws/path/gymclockwise.csv')
original_x = df['x'].values
original_y = df['y'].values

# --- 1. Identify Bad Segments (Weave Detection) ---
# Calculate headings
diffs = df[['x', 'y']].diff().iloc[1:]
diffs['heading'] = np.arctan2(diffs['y'], diffs['x'])

# Re-index
df['heading'] = np.nan
df.iloc[1:, df.columns.get_loc('heading')] = diffs['heading']

# Angular diff function
def angular_diff(a, b):
    diff = a - b
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff

# Delta heading
heading_diffs = [0]
headings = df['heading'].values
for i in range(1, len(headings)):
    if np.isnan(headings[i]) or np.isnan(headings[i-1]):
        heading_diffs.append(0)
    else:
        heading_diffs.append(angular_diff(headings[i], headings[i-1]))
df['delta_heading'] = heading_diffs

# Weave Score
window_size = 21
half_window = window_size // 2
weave_scores = np.zeros(len(df))

for i in range(half_window, len(df) - half_window):
    window_deltas = df['delta_heading'].iloc[i - half_window : i + half_window].values
    total_abs_turn = np.sum(np.abs(window_deltas))
    net_turn = np.abs(np.sum(window_deltas))
    weave_scores[i] = total_abs_turn - net_turn

# Smooth score and threshold
score_series = pd.Series(weave_scores)
score_smooth = score_series.rolling(5, center=True).mean().fillna(0)
threshold = score_smooth.mean() + 2 * score_smooth.std()

# Identify indices to smooth
# We create a mask: 1 where smoothing is needed, 0 otherwise
mask = (score_smooth > threshold).astype(float).values

# Dilate the mask to include transition zones (e.g. +/- 10 points)
# This ensures we don't just smooth the peak but the whole wobble
from scipy.ndimage import binary_dilation
mask_dilated = binary_dilation(mask > 0, iterations=10).astype(float)

# Create a smooth transition weight (ramp)
# We can use a gaussian filter on the binary mask to create slopes
from scipy.ndimage import gaussian_filter1d
# sigma=5 gives a nice ramp over ~10-20 points
weights = gaussian_filter1d(mask_dilated, sigma=5)
# Clip to [0, 1] just in case, though gaussian on 0/1 usually stays inside
weights = np.clip(weights, 0, 1)

# --- 2. Generate Globally Smoothed Path ---
# We use a stronger filter for the "target" smoothed path
# Window 51 is approx 25m, good for fixing weaves
sg_window = 51
sg_poly = 3
if len(df) > sg_window:
    x_global_smooth = savgol_filter(original_x, window_length=sg_window, polyorder=sg_poly)
    y_global_smooth = savgol_filter(original_y, window_length=sg_window, polyorder=sg_poly)
else:
    x_global_smooth = original_x
    y_global_smooth = original_y

# --- 3. Blend ---
# Final = Original * (1 - w) + Smooth * w
x_final = original_x * (1 - weights) + x_global_smooth * weights
y_final = original_y * (1 - weights) + y_global_smooth * weights

# --- 4. Uniform Density Sampling ---
# Calculate cumulative distance along the path
dx = np.diff(x_final)
dy = np.diff(y_final)
seg_len = np.hypot(dx, dy)
cum_dist = np.insert(np.cumsum(seg_len), 0, 0.0)

# Create uniformly spaced distances
target_spacing = 0.7
sampled_dists = np.arange(0, cum_dist[-1], target_spacing)

# Ensure the last point is included if it's not already
if len(sampled_dists) == 0 or sampled_dists[-1] < cum_dist[-1]:
    sampled_dists = np.append(sampled_dists, cum_dist[-1])

# Interpolate to get new x and y
from scipy.interpolate import interp1d
interp_x = interp1d(cum_dist, x_final, kind='linear')
interp_y = interp1d(cum_dist, y_final, kind='linear')

dense_x = interp_x(sampled_dists)
dense_y = interp_y(sampled_dists)

# Save result
df_result = pd.DataFrame({
    'x': x_final,
    'y': y_final
})
df_result.to_csv('turn_2ms_smoothed.csv', index=False)

df_dense = pd.DataFrame({
    'x': dense_x,
    'y': dense_y
})
df_dense.to_csv('/home/cyc/campus_ws/path/smoothed/gymclockwise.csv', index=False)

# Find segments where weights > 0.1
changed_indices = np.where(weights > 0.1)[0]
if len(changed_indices) > 0:
    plt.scatter(x_final[changed_indices], y_final[changed_indices], 
                c='blue', s=2, alpha=0.5, label='Smoothed Segments')

# Zoom in on the worst segment (Index 1201-1210 detected previously)
# Let's zoom around 1200
zoom_idx = 1205
zoom_range = 50
start_z = max(0, zoom_idx - zoom_range)
end_z = min(len(df), zoom_idx + zoom_range)

# Inset axes for zoom
# We'll just do a separate plot or a subplot
# Let's do a 2-panel plot
plt.title('Whole Path with Selective Smoothing')
plt.legend()
plt.axis('equal')
plt.savefig('selective_smoothing_overview.png')

plt.figure(figsize=(12, 6))
plt.plot(original_x[start_z:end_z], original_y[start_z:end_z], 'k.-', label='Original', alpha=0.5)
plt.plot(x_final[start_z:end_z], y_final[start_z:end_z], 'r.-', label='Smoothed', linewidth=2)
plt.title(f'Zoomed View (Index {start_z}-{end_z})')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.savefig('selective_smoothing_zoom.png')

print(f"Smoothed {np.sum(mask)} points (raw detected), affected {np.sum(weights > 0.01)} points with blending.")
print(f"Dense output points: {len(dense_x)} (original smoothed points: {len(x_final)}), spacing=0.5m")

