"""
run zoedepth on those 20 frames, linear fits scale/offset, saves plot
"""

import numpy as np
from PIL import Image
from depth import DepthEstimator

# load model once with no calib
est = DepthEstimator(scale=1.0, offset=0.0)

results = []

for i in range(20):
    color_path = f'/home/roboticslab/Documents/ellen/calibration_frames/color_{i:02d}.png'
    depth_path = f'/home/roboticslab/Documents/ellen/calibration_frames/depth_{i:02d}.npy'
    
    try:
        color_img = np.array(Image.open(color_path))
        true_depth = np.load(depth_path)

        pred_depth = est.estimate(color_img)
        pred_resized = np.array(Image.fromarray(pred_depth).resize((640,400)))

        # center region only
        h,w = true_depth.shape
        mh,mw = h//4, w//4
        true_center = true_depth[mh:h-mh, mw:w-mw]
        pred_center = pred_resized[mh:h-mh, mw:w-mw]

        # filter zeros (invalid readings)
        mask = true_center > 0
        if mask.sum() < 100: # less than 100 total valid pxls
            print(f"frame {i} not enough valid pxls")
            continue

        true_vals = true_center[mask] / 1000.0 # mm to m
        pred_vals = pred_center[mask]

        # sample 1000 pts
        if len(true_vals) > 1000:
            idx = np.random.choice(len(true_vals), 1000, replace=False)
            true_vals = true_vals[idx]
            pred_vals = pred_vals[idx]

        results.append((pred_vals, true_vals))
    
    except Exception as e:
        print(f"frame {i} error {e}")

# filter to reliable depth range (0.3m to 1.5m)
filtered_results = []
for pred_vals, true_vals in results:
    mask = (true_vals > 0.3) & (true_vals < 1.5)
    if mask.sum() > 50:
        filtered_results.append((pred_vals[mask], true_vals[mask]))

all_pred = np.concatenate([r[0] for r in filtered_results])
all_true = np.concatenate([r[1] for r in filtered_results])

# true = pred*scale + offset
coeffs = np.polyfit(all_pred, all_true, 1)
scale, offset = coeffs[0], coeffs[1]

# r sqrd
pred_fitted = all_pred * scale + offset
ss_res = np.sum((all_true - pred_fitted)**2)
ss_tot = np.sum((all_true - all_true.mean()) **2)
r_sqrd = 1 - (ss_res/ss_tot)

# rmse
rmse = np.sqrt(np.mean((all_true - pred_fitted) ** 2))

print(f"scale: {scale:.4f}")
print(f"offset: {offset:.4f}")
print(f"r sqrd: {r_sqrd:.4f}")
print(f"rmse: {rmse:.4f}m ({rmse*100:.1f}cm)")

# delete later
# === Diagnostic plots ===
import matplotlib.pyplot as plt

# 1. Scatter plot: predicted vs true
plt.figure(figsize=(12, 4))

plt.subplot(1, 3, 1)
plt.scatter(all_pred, all_true, alpha=0.1, s=1)
plt.plot([0, 3], [0*scale+offset, 3*scale+offset], 'r-', label='Linear fit')
plt.xlabel('ZoeDepth prediction (m)')
plt.ylabel('True depth (m)')
plt.title('Predicted vs True')
plt.legend()

# 2. Error distribution
errors = all_true - (all_pred * scale + offset)
plt.subplot(1, 3, 2)
plt.hist(errors, bins=50)
plt.xlabel('Error (m)')
plt.ylabel('Count')
plt.title(f'Error distribution\nmean={errors.mean():.3f}, std={errors.std():.3f}')

# 3. Error vs depth (is error worse at certain distances?)
plt.subplot(1, 3, 3)
plt.scatter(all_true, errors, alpha=0.1, s=1)
plt.axhline(y=0, color='r', linestyle='--')
plt.xlabel('True depth (m)')
plt.ylabel('Error (m)')
plt.title('Error vs True Depth')

plt.tight_layout()
plt.savefig('/home/roboticslab/Documents/ellen/calibration_analysis.png')
plt.show()
print("Saved calibration_analysis.png")