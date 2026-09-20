"""
pulls 20 paired color/IR frames out of bag - already run, dont need again
"""

import sqlite3
import numpy as np
from PIL import Image
import os
import struct

# create output folder
os.makedirs('calibration_frames', exist_ok=True)

# open bag db
db = sqlite3.connect('calibration_bag/calibration_bag_0.db3') # creates connection obj, opens comm to db file
cursor = db.cursor() # cursor is active ptr, used to send cmds and fetch results

# get topic IDs
cursor.execute('SELECT id, name FROM topics') # want id and name cols, from topics table
topics = {name: id for id, name in cursor.fetchall()}
print(f"topics found: {topics}")

color_topicid = topics.get('/camera/color/image_raw')
depth_topicid = topics.get('/camera/depth/image_raw')

if not color_topicid or not depth_topicid:
    print("error missing topics")
    exit(1)

# get msgs
cursor.execute(f"SELECT COUNT(*) FROM messages WHERE topic_id = {color_topicid}") # count num of rows
total_color = cursor.fetchone()[0] # returns actual number. fetchone gets first avail row, [0] gets item from tuple
print(f"total color frames: {total_color}")

cursor.execute(f"SELECT COUNT(*) FROM messages WHERE topic_id = {depth_topicid}")
total_depth = cursor.fetchone()[0]
print(f"total depth frames: {total_depth}")

# extract 20 evenly spaced frames
num_frames = 20
step = total_color // num_frames

for i in range(num_frames):
    offset = i*step

    # get color frame
    cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {color_topicid} LIMIT 1 OFFSET {offset}")
    color_row = cursor.fetchone()

    # get depth frame closest in time
    color_timestmp = color_row[0]
    cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {depth_topicid} AND timestamp >= {color_timestmp} LIMIT 1")
    depth_row = cursor.fetchone()

    if not depth_row:
        print(f"no matching depth frame")
        continue

    # parse color image
    color_data = color_row[1]
    try:
        # ROS msg has header, try to find img data
        img_size = 640*480*3
        color_img = np.frombuffer(color_data[-img_size:], dtype=np.uint8).reshape(480,640,3)
        Image.fromarray(color_img).save(f'calibration_frames/color_{i:02d}.png')
    except Exception as e:
        print(f"color parse error {e}")
        continue

    # parse depth image
    depth_data = depth_row[1]
    try:
        depth_size = 640*400*2
        depth_img = np.frombuffer(depth_data[-depth_size:], dtype=np.uint16).reshape(400,640)

        # save raw depth as np
        np.save(f'calibration_frames/depth_{i:02d}.npy', depth_img)

        # also save visualization
        depth_vis = (depth_img / depth_img.max() * 255).astype(np.uint8) # conv to 0 to 1 range then 0 to 255 range
        Image.fromarray(depth_vis).save(f'calibration_frames/depth_{i:02d}.png')

        print(f"frame {i} saved (depth range {depth_img.min()} to {depth_img.max()} mm)")
    
    except Exception as e:
        print(f"depth parse error {e}")
        continue

db.close()
print("done")
