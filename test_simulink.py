from comm.Simulink import Simulink
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os
import json
import yaml
import json
from time import sleep

config_path = './config.YAML'
coord_path = './calibration/captures'

#   load config
with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    simulinkConf = config['SIMULINK']

#   load panda coordinates
with open(f"{coord_path}/calibration_points.json", 'r') as file:
    panda_coords = json.load(file)

"""
#   RealSense Intialization
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipe.start(cfg)

for x in range(20):
    pipe.wait_for_frames()
"""

path = './calibration/captures'
name = input("Name: ")
robot_coord = []
simulink = Simulink(simulinkConf['ENGINE_NAME'], simulinkConf['SIMULINK_SESSION_NAME'])

#   initialize control attribute
simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '0')
    
for panda_coord in panda_coords:
    
    #   set coordinates and start movement
    simulink.setAttribute(simulinkConf['COORDINATE_ATTRIBUTE'], 'Value', panda_coord)
    
    sleep(0.1) #    check if this is really needed
    
    print(f"Sending signal to move to {panda_coord}")
    
    simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '1')
    
    input('Arrived at Goal?') # needed since movement takes unknown amount of time, could be automated with upper boundary
    
    simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '0')
    
    sleep(0.1)
    
    items = os.listdir(f'{path}/rgb')
    files = [item for item in items if os.path.isfile(os.path.join(f'{path}/rgb', item))]
    run = len(files) + 1
    """
    #   capture image from Realsense Camera
    frameset = pipe.wait_for_frames()

    rgb = Image.fromarray(np.asanyarray(frameset.get_color_frame().get_data()))
    depth = Image.fromarray(np.asanyarray(frameset.get_depth_frame().get_data()))

    rgb.save(f'{path}/rgb/{name}{run}.png')
    depth.save(f'{path}/depth/{name}{run}.png')
    """

#pipe.stop()
