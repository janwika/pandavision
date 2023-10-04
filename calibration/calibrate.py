import os
import open3d as o3d
import json
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from tempfile import TemporaryFile

# read depth, rgb from Images

skip = [3, 11, 29, 30, 34, 37, 38, 39, 41, 42, 43, 46, 47, 48]

path = f"{os.getcwd()}/calibration/captures"
rgb = []
depth = []
debug_2d = False
debug_3d = False

file_list = sorted(os.listdir(f"{path}/rgb"))

for filename in file_list:
    rgb.append(o3d.io.read_image(os.path.join(f"{path}/rgb", filename)))
    depth.append(o3d.io.read_image(os.path.join(f"{path}/depth", filename)))

# parse panda coordinates from json

with open(f"{path}/calibration_points.json", 'r') as file:
    panda_coords = json.load(file)
    
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
    
for i in range(len(panda_coords)):
	val = panda_coords[i]
	val = val.replace(";", ",")
	coor = eval(val)
	coor[2] = coor[2] - 0.1 # improve this value! (offset to aruco center)
	panda_coords[i] = coor
	
print(panda_coords)
deleted = 0
# detect aruco corners and calculate center
cam_coords = []
for i in range(len(rgb)):

    if(i+1 in skip):
        print("Skipping iteration", i)
        del panda_coords[i - deleted]
        deleted += 1
        continue
    print(i)

    d = depth[i]
    d_np = np.asarray(d)
    r = rgb[i]
    r_np = np.asarray(r)
    
    gray = cv2.cvtColor(r_np, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    
    corners, _, _= detector.detectMarkers(gray)

    corner_matrix = corners[0][0]
    
    # visual debugging 2d image
    if(debug_2d):
        draw_corner = aruco.drawDetectedMarkers(gray, corners)
        plt.scatter(corner_matrix[0][0], corner_matrix[0][1], s=1, marker='.', color="red")
        plt.scatter(corner_matrix[1][0], corner_matrix[1][1], s=1, marker='x', color="red")
        plt.scatter(corner_matrix[2][0], corner_matrix[2][1], s=1, marker='x', color="red")
        plt.scatter(corner_matrix[3][0], corner_matrix[3][1], s=1, marker='x', color="red")
        plt.imshow(draw_corner, cmap='gray', vmin=0, vmax=255)
        plt.show()

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
    center = (midpoint1 + midpoint2) / 2
    
    if(len(pc.points) == 0):
        print("Could not find 3d point for", i)
        del panda_coords[i]
        continue
    
    cam_coords.append(center)

    # visual debugging 3d pointcloud
    if(debug_3d):
        center_and_corners = np.vstack((corners_3d, np.array(center)))
        pc.points = o3d.utility.Vector3dVector(center_and_corners)

        pc.paint_uniform_color([1.0, 0, 0])

        # Project center point into pointcloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(r, d)
            
        pc_e = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd,
            intrinsic
        )
        pc_e.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

        o3d.visualization.draw_geometries([pc_e, pc, coordinate_frame])
        
    
    
#   at this point panda_coords holds the robot coordinates and cam_coords holds the center coords of the aruco code

# prepare matrices
panda_coords = np.array(panda_coords)
cam_coords = np.array(cam_coords)


for i in range(len(panda_coords)):
	print(f"panda: {panda_coords[i]}; cam: {cam_coords[i]}")



# estimate camera to panda translation/rotation matrix
retval, affine_matrix, _ = cv2.estimateAffine3D(cam_coords, panda_coords)

print(f"Estimation Successful: {retval}")
print("\nEstimated Affine Matrix:")
print(affine_matrix)

# Reproject camera coordinates back to panda coordinates using the transformation matrix
reprojected_panda_coords = np.dot(cam_coords - affine_matrix[:3, 3], np.linalg.inv(affine_matrix[:3, :3]).T)

# Calculate the error for each axis
error = reprojected_panda_coords - panda_coords

for i in range(len(panda_coords)):
	print(f"{i}: panda: {panda_coords[i]}; cam coords: {cam_coords[i]}; error: {error[i]}")
    
mean_error = np.mean(error, axis=0)
median_error = np.median(error, axis=0)


calibration_matrix = affine_matrix.reshape(3,4)
calibration_matrix = np.vstack((calibration_matrix, [0, 0, 0, 1]))
print(calibration_matrix)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb[0], depth[0])
pc_e = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
pc_e.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
pc_e.transform(calibration_matrix)

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

o3d.visualization.draw_geometries([pc_e, pc, coordinate_frame])



print("\nReprojection Result:")
print("Mean Error:", mean_error)
print("Median Error:", median_error)

print("\nSaving matrix to ./calibration_matrix.csv")

affine_matrix.tofile('calibration_matrix.csv',sep=',',format='%10.16f')
