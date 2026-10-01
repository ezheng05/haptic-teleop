"""
core depth estimation functions
Python functions for depth est, no ROS
for standalone testing

usage:
    as module
        - from depth import DepthEstimator, find_closest
        - estimator = DepthEstimator()
        - depth_map = estimator.estimate(image)
        - result = find_closest(depth_map)

    standalone
        - python3 depth.py test_image.jpg
"""

import numpy as np
import torch
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

# linear fit, 19 frames vs on-board IR (Sep 2026)
DEPTH_SCALE = 0.2526
DEPTH_OFFSET = 0.0195

# quadratic refit, 7 tape-measured positions 0.30-1.50 m (24 Sep 2026)
#   d = A*raw^2 + B*raw + C       rmse 2.3 cm, vs 18.7 cm for the linear fit
# the parabola's vertex is at raw = -B/(2A) ~ 1.21; below that it turns back
# UP, so a very close object would read as farther. below RAW_MIN the curve
# is continued downward with the linear fit's slope (see calibrate()); above
# RAW_MAX it is held flat. keeps the mapping monotone and conservative.
DEPTH_QUAD = (0.1196, -0.2886, 0.4771)
RAW_MIN, RAW_MAX = 1.21, 4.50

class DepthEstimator:
    """
    wrapper for ZoeDepth
    RGB -> depth map
    each pixel value = dist from camera in m
    """

    def __init__(self, device=None, scale=None, offset=None, calib=None):
        # init depth estimator
        # args: device: cuda for GPU, cpu, or None for auto detect
        #       calib:  'quadratic' | 'linear' | 'raw'
        #               default is 'linear' when scale/offset are passed
        #               (so older callers keep working), else 'quadratic'
        if calib is None:
            calib = 'linear' if (scale is not None or offset is not None) \
                else 'quadratic'
        if scale is None:
            scale = DEPTH_SCALE
        if offset is None:
            offset = DEPTH_OFFSET

        if device is None:
            if torch.cuda.is_available():
                device = 'cuda'
            elif torch.backends.mps.is_available():
                device = 'mps'
            else:
                device = 'cpu'
        self.device = torch.device(device)
        self.scale = scale
        self.offset = offset
        self.calib = calib

        # load model from hugging face, downloads model weights on first run
        model_name = "Intel/zoedepth-nyu-kitti"
        self.processor = AutoImageProcessor.from_pretrained(model_name) # auto sets rules/configurations for preparing/formatting images to match model requirements/configs
        self.model = AutoModelForDepthEstimation.from_pretrained(model_name) # loads model for depth est
        self.model = self.model.to(self.device) # moves model to GPU
        self.model.eval() # evaluation/inference mode, not training

    def estimate(self, image):
        # est depth from RGB image
        # args: image: numpy array (H,W,3) - RGB, 0-255, or PIL # 3 is number of channels
        # returns depth map: numpy array (H,W) - depth in m

        # convert numpy to PIL if needed
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image.astype(np.uint8))

        # preprocess: resize + normalize for NN
        inputs = self.processor(images=image, return_tensors="pt") # returns pytorch tensor
        inputs = inputs.to(self.device)

        # run NN 
        with torch.no_grad(): # context manager: no grad while within 'with'
            outputs = self.model(**inputs) # ** unpacks input
            predicted_depth = outputs.predicted_depth

        # model outputs small depth map, resize to original image size
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1), # add channel dim
            size=image.size[::-1], # reverses: w,h -> height, width
            mode="bicubic", # to stretch image from small to large: take avg of 16 nearby pxls -> smoothest
            align_corners=False, # determines if corner pixels stay on corners
        )

        """
        interpolate func expects 4D batch format: [batch, channels, height, width]
        unsqueeze(1) adds dim at index 1: [batch, h, w] -> [batch, channel, h, w]
        """

        # convert from pytorch tensor to numpy arr
        depth_map = prediction.squeeze().cpu().numpy()
        return self.calibrate(depth_map)

    def calibrate(self, raw):
        # raw model output -> metres, per self.calib
        if self.calib == 'raw':
            return raw
        if self.calib == 'quadratic':
            a, b, c = DEPTH_QUAD
            r = np.minimum(raw, RAW_MAX)
            d = a * r * r + b * r + c
            # below the vertex the parabola turns back up, which would make a
            # very close object read as farther. continue downward from the
            # vertex with the linear fit's slope instead, so closer stays
            # closer and the barrier still trips.
            d_vertex = a * RAW_MIN * RAW_MIN + b * RAW_MIN + c
            d_lo = d_vertex + DEPTH_SCALE * (r - RAW_MIN)
            return np.maximum(np.where(r < RAW_MIN, d_lo, d), 0.05)
        return raw * self.scale + self.offset

def find_closest(depth_map, margin=50, bottom_frac=0.35):
    """
    find closest pt in depth map
    ignore edges because often incorrect val at borders

    args:
        depth_map: numpy arr (H,W)
        margin: pixels to ignore at each edge
    
    returns:
        dict with: 
            depth - dist to closest
            x,y - pxl coord of closest
            direc - left, center, or right
    """
    h,w = depth_map.shape
    bottom = int(h*bottom_frac)
    inner = depth_map[margin:h-bottom, margin:w-margin]

    min_depth = float(np.min(inner))

    # find where min is (row, col)
    min_idx = np.unravel_index(np.argmin(inner), inner.shape)
    # argmin gives position without accounting for shape, unravel converts list position back into r,c

    # convert back to full img coord
    y = min_idx[0] + margin # row = y
    x = min_idx[1] + margin # col = x

    # determine direc
    if x < w/3:
        direc = 'left'
    elif x > 2*w/3:
        direc = 'right'
    else:
        direc = 'center'
    
    return {
        'depth': min_depth,
        'x': x,
        'y': y,
        'direction': direc
    }
    

# standalone test without ROS: python3 depth.py test_image.jpg

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python3 depth.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]
    print(f"Loading image: {image_path}")

    # load img
    img = Image.open(image_path)
    img_np = np.array(img)
    print(f"Image size: {img_np.shape}")

    # create estimator
    print("Loading model")
    estimator = DepthEstimator()
    print(f"Using device: {estimator.device}")

    # run estimation
    print("Estimating...")
    depth_map = estimator.estimate(img_np)
    
    # find closest
    result = find_closest(depth_map, margin=50)

    print(f"\nResults:")
    print(f"Closest point: {result['depth']:.2f} m")
    print(f"Location: ({result['x']}, {result['y']})")
    print(f"Direction: {result['direction']}")
    print(f"Range: {depth_map.min():.2f} m to {depth_map.max():.2f} m")

    # save
    depth_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
    depth_img = Image.fromarray((depth_normalized*255).astype(np.uint8))
    output_path = image_path.rsplit('.',1)[0] + '_depth.png'
    depth_img.save(output_path)
    print(f"Saved to {output_path}")