"""Closest-point error: the metric cbf_node actually consumes."""
import numpy as np, os
from PIL import Image
from depth import DepthEstimator

FRAMES = os.path.expanduser('~/Documents/ellen/calibration_frames')
SCALE, OFFSET, MARGIN = 0.2526, 0.0195, 50

est = DepthEstimator(scale=SCALE, offset=OFFSET)
rows = []

for i in range(20):
    try:
        color = np.array(Image.open(f'{FRAMES}/color_{i:02d}.png'))
        true = np.load(f'{FRAMES}/depth_{i:02d}.npy') / 1000.0
    except Exception as e:
        print(f"frame {i}: {e}"); continue

    pred = est.estimate(color)
    pred = np.array(Image.fromarray(pred).resize((640, 400)))

    h, w = true.shape
    ip = pred[MARGIN:h-MARGIN, MARGIN:w-MARGIN]
    it = true[MARGIN:h-MARGIN, MARGIN:w-MARGIN]

    valid = it > 0
    if valid.sum() < 100:
        print(f"frame {i}: too few valid"); continue

    d_pred = float(ip.min())
    d_true = float(np.percentile(it[valid], 1))   # 1st pct, robust to dead pixels
    rows.append((i, d_pred, d_true, d_pred - d_true))
    print(f"frame {i:2d}  pred {d_pred:.3f}  ref {d_true:.3f}  err {d_pred-d_true:+.3f} m")

e = np.array([r[3] for r in rows])
print(f"\nn = {len(e)}")
print(f"mean error   {e.mean()*100:+.1f} cm")
print(f"median |err| {np.median(abs(e))*100:.1f} cm")
print(f"RMSE         {np.sqrt((e**2).mean())*100:.1f} cm")
np.savez(os.path.expanduser('~/Documents/ellen/closest_error.npz'),
         rows=np.array(rows))
