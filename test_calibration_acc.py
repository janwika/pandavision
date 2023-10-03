import os
import open3d as o3d
import pyrealsense2 as rs
import json
import yaml
import numpy as np
import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from time import sleep
from tempfile import TemporaryFile
from comm.Simulink import Simulink


config_path = './config.YAML'
path = f"{os.getcwd()}/calibration/captures"

with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    simulinkConf = config['SIMULINK']

with open(f"{path}/intrinsics.json", 'r') as file:
    intrinsic_params = json.load(file)
    
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    intrinsic_params["width"],
    intrinsic_params["height"],
    intrinsic_params["fx"],
    intrinsic_params["fy"],
    intrinsic_params["ppx"],
    intrinsic_params["ppy"]
)
    
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
    
# Get frameset of color and depth
frames = pipeline.wait_for_frames()

# Align the depth frame to color frame
aligned_frames = align.process(frames)

# Get aligned frames
d = aligned_frames.get_depth_frame()
d_np = np.asarray(d)
r = aligned_frames.get_color_frame()
r_np = np.asarray(r)

gray = cv2.cvtColor(r_np, cv2.COLOR_BGR2GRAY)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

corners, _, _= detector.detectMarkers(gray)

corner_matrix = corners[0][0]

corner_depth = np.full((d_np.shape[0], d_np.shape[1]), -1).astype(np.uint16)

for corner in corner_matrix:
    z = d_np[round(corner[1])][round(corner[0])]
    corner_depth[round(corner[1])][round(corner[0])] = d_np[round(corner[1])][round(corner[0])]

# Project center point into pointcloud
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(r, o3d.geometry.Image((corner_depth).astype(np.uint16)))
    
pc = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd,
    intrinsic,
    project_valid_depth_only = True
)

# Rotation needed because of Pinhole Projection
pc.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

corners_3d = np.asarray(pc.points)

print(corners_3d)

# Calculate the midpoints of the diagonals
midpoint1 = (corners_3d[0] + corners_3d[2]) / 2
midpoint2 = (corners_3d[1] + corners_3d[3]) / 2

# Calculate the center by averaging the two midpoints
observed_coord = (midpoint1 + midpoint2) / 2
    
    
simulink = Simulink(simulinkConf['ENGINE_NAME'], simulinkConf['SIMULINK_SESSION_NAME'])

simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '0')

simulink.setAttribute(simulinkConf['COORDINATE_ATTRIBUTE'], 'Value', observed_coord)

sleep(0.1) #    check if this is really needed

print(f"Sending signal to move to {observed_coord}")

simulink.setAttribute(simulinkConf['ACTIVATE_ATTRIBUTE'], 'Value', '1')

input('Arrived?')

real_coord = simulink.getRealTimeValue(1)

# TODO: get 3d Vector of coords here

print(f"Observed {observed_coord}; arrived at {real_coord}")
print(f"Error: {observed_coord - real_coord}")