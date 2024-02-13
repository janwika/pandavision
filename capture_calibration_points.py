from comm.Simulink import Simulink
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os
import json
import yaml
import json
from enum import IntEnum
from time import sleep

# enum for realsense presets
class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5

config_path = './config.YAML'
path = './calibration/captures'

# saves camera intrinsic parameters to json file
def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'fx':
                    intrinsics.fx,
                'fy':
                    intrinsics.fy,
                'ppx':
                    intrinsics.ppx,
                'ppy':
                    intrinsics.ppy
            },
            outfile,
            indent=4)

#   load config
with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    simulinkConf = config['SIMULINK']

#   load panda coordinates
with open(f"{path}/calibration_points.json", 'r') as file:
    panda_coords = json.load(file)

#   RealSense Intialization
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()

#depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

align_to = rs.stream.color
align = rs.align(align_to)

for x in range(20):
    pipeline.wait_for_frames()

name = input("Name: ")
simulink = Simulink(simulinkConf['ENGINE_NAME'], simulinkConf['SIMULINK_SESSION_NAME'])

#   initialize control attribute
simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '0')
    
for panda_coord in panda_coords:
    #   set coordinates and start movement
    simulink.setAttribute(simulinkConf['COORDINATE_HOME_ATTRIBUTE'], 'Value', panda_coord)
    
    sleep(0.1) #    check if this is really needed
    
    print(f"Sending signal to move to {panda_coord}")
    
    simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '1')
    
    input('Arrived at Goal?') # needed since movement takes unknown amount of time, could be automated with upper boundary
    #   figure out name of files
    items = os.listdir(f'{path}/rgb')
    files = [item for item in items if os.path.isfile(os.path.join(f'{path}/rgb', item))]
    run = len(files) + 1
    
    simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '0')
    
    #   capture image from Realsense Camera
    
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    
    rgb = Image.fromarray(color_image)
    depth = Image.fromarray(depth_image)
    
    padded_run = str(run).zfill(3)

    rgb.save(f'{path}/rgb/{name}{padded_run}.png')
    depth.save(f'{path}/depth/{name}{padded_run}.png')

pipeline.stop()
save_intrinsic_as_json(f"{path}/intrinsics.json", color_frame)
