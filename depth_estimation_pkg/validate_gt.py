"""Validate ZoeDepth against tape measured ground truth"""

import os, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from depth import DepthEstimator

VAL = os.path.expanduser('~/Documents/ellen/validation')
SCALE, OFFSET = 0.2526, 0.0195
TRUE = {'val_030': 0.30, 'val_040': 0.40, 'val_050': 0.50, 'val_075': 0.75,
        'val_100': 1.00, 'val_125': 1.25, 'val_150': 1.50}
COLOR_T, DEPTH_T = '/camera/color/image_raw', '/camera/depth/image_raw'


def to_np(msg):
    a = np.frombuffer(msg.data, dtype=np.uint16 if '16' in msg.encoding
                      else np.uint8)
    ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
    a = a.reshape(msg.height, msg.width, ch).squeeze()
    if msg.encoding == 'bgr8':
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


def middle_pair(bagdir):
    r = SequentialReader()
    r.open(StorageOptions(uri=bagdir, storage_id='sqlite3'),
           ConverterOptions('', ''))
    col, dep = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == COLOR_T:
            col.append(data)
        elif topic == DEPTH_T:
            dep.append(data)
    if not col or not dep:
        return None, None
    return (deserialize_message(col[len(col) // 2], Image),
            deserialize_message(dep[len(dep) // 2], Image))


est = DepthEstimator(scale=SCALE, offset=OFFSET)
print(f"{'bag':10} {'tape':>6} {'zoe':>7} {'IR':>7} {'zoe-tape':>9} {'IR-tape':>8}")
rows = []

for name in sorted(TRUE):
    bag = os.path.join(VAL, name)
    if not os.path.isdir(bag):
        print(f"{name:10} MISSING"); continue
    cmsg, dmsg = middle_pair(bag)
    if cmsg is None:
        print(f"{name:10} no frames"); continue

    color = to_np(cmsg)
    true_m = to_np(dmsg).astype(np.float32) / 1000.0
    pred = est.estimate(color)
    pred = np.array(PILImage.fromarray(pred).resize(
        (true_m.shape[1], true_m.shape[0])))

    h, w = true_m.shape
    cy, cx = h // 2, w // 2
    pp = pred[cy-40:cy+40, cx-60:cx+60]
    pt = true_m[cy-40:cy+40, cx-60:cx+60]
    valid = pt > 0

    d_zoe = float(np.median(pp))
    d_ir = float(np.median(pt[valid])) if valid.sum() > 50 else float('nan')
    t = TRUE[name]
    rows.append((t, d_zoe, d_ir))
    print(f"{name:10} {t:6.2f} {d_zoe:7.3f} {d_ir:7.3f} "
          f"{d_zoe-t:+9.3f} {d_ir-t:+8.3f}")

a = np.array(rows)
ez, ei = a[:, 1] - a[:, 0], a[:, 2] - a[:, 0]
print(f"\nZoeDepth vs tape : median|e| {np.median(abs(ez))*100:5.1f} cm   "
      f"RMSE {np.sqrt(np.nanmean(ez**2))*100:5.1f} cm")
print(f"IR vs tape       : median|e| {np.nanmedian(abs(ei))*100:5.1f} cm   "
      f"RMSE {np.sqrt(np.nanmean(ei**2))*100:5.1f} cm")
np.savez(os.path.join(VAL, 'validation.npz'), rows=a)
print(f"\nsaved {VAL}/validation.npz")
