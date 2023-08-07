import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipe.start(cfg)

for x in range(20):
    pipe.wait_for_frames()
    
path = './finetuning_data/cutlery/top_down'
name = input("Name: ")
    
while(True):
    capture = input("Capture? [*|n]: ")
    if(capture == 'n'): break
    
    items = os.listdir(f'{path}/rgb')
    files = [item for item in items if os.path.isfile(os.path.join(f'{path}/rgb', item))]
    run = len(files) + 1
    
    frameset = pipe.wait_for_frames()

    rgb = Image.fromarray(np.asanyarray(frameset.get_color_frame().get_data()))
    depth = Image.fromarray(np.asanyarray(frameset.get_depth_frame().get_data()))

    rgb.save(f'{path}/rgb/{name}{run}.png')
    depth.save(f'{path}/depth/{name}{run}.png')

pipe.stop()