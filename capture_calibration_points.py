from comm.Simulink import Simulink
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os
import json
import yaml

config_path = './config.YAML'

with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    simulinkConf = config['SIMULINK']

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipe.start(cfg)

for x in range(20):
    pipe.wait_for_frames()
    
path = './calibration/captures'
name = input("Name: ")
robot_coord = []
s = Simulink(simulinkConf['ENGINE_NAME'], simulinkConf['SIMULINK_SESSION_NAME'])
    
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
    hand_coord = s.getAttribute(simulinkConf['READ_ATTRIBUTE'])
    
    print(f"Captured RGB, Depth at panda position {hand_coord}")
    
    robot_coord.append({'id': f'{name}{run}', 'coord': f'{hand_coord}'})
    
json_object = json.dumps(robot_coord, indent=4)
with open(f"{path}/calibration_points.json", "w") as outfile:
    outfile.write(json_object)

pipe.stop()