import os
import open3d as o3d
import json
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from tempfile import TemporaryFile

# read depth, rgb from Images

path = f"{os.getcwd()}/calibration/captures"
rgb = []
depth = []
debug = True

file_list = os.listdir(f"{path}/rgb")

for filename in file_list:
    rgb.append(o3d.io.read_image(os.path.join(f"{path}/rgb", filename)))
    depth.append(o3d.io.read_image(os.path.join(f"{path}/depth", filename)))

# parse panda coordinates from json

with open(f"{path}/calibration_points.json", 'r') as file:
    data = json.load(file)

panda_coords = []
for item in data:
    try:
        coord_list = json.loads(item['coord'])  # Convert the string to a list
        panda_coords.append(coord_list)
    except json.JSONDecodeError:
        print(f"Invalid 'coord' value in entry with id: {item['id']}")

# detect aruco corners and calculate center
aruco_centers = []
for i in range(len(rgb)):
    image = rgb[i]
    rgb_np = np.asarray(image)
    
    gray = cv2.cvtColor(rgb_np, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    center = np.average(corners[0][0], axis=0)
    aruco_centers.append(center)
    
    draw_corner = aruco.drawDetectedMarkers(gray, corners)
    
    if(debug):
        plt.plot(center[0], center[1], marker='x', color="red")
        plt.imshow(draw_corner, cmap='gray', vmin=0, vmax=255)
        plt.title(f"Image {i}, Center at {center}")
        plt.get_current_fig_manager().full_screen_toggle()
        plt.show()

cam_coords = []
# use open3d pointcloud to calculate center coordinates
for i in range(len(aruco_centers)):
    cen = aruco_centers[i]
    d = depth[i]
    r = rgb[0]

    d_np = np.asarray(d)
    
    z = d_np[int(cen[1])][int(cen[0])] # get depth with floored x,y of aruco center
    
    cen = np.append(cen, z)
    
    mat = np.full((d_np.shape[0], d_np.shape[1]), -1).astype(np.uint16)

    mat[int(cen[1])][int(cen[0])] = int(cen[2])
    
    # Project center point into pointcloud
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(r, o3d.geometry.Image((mat).astype(np.uint16)))
        
    pc = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
        ),
        project_valid_depth_only = True
    )
    
    # Rotation needed because of Pinhole Projection
    pc.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    cam_coords.append(pc.points[0])
    
    
#   at this point panda_coords holds the robot coordinates and cam_coords holds the center coords of the aruco code

# prepare matrices
panda_coords = np.array(panda_coords)
cam_coords = np.array(cam_coords)



# TEST DATA; MUST DELETE
"""
np.random.seed(42)
num_points = 8
panda_coords = np.random.rand(num_points, 3)
rotation_matrix = np.array([[0.866, -0.5, 0],
                            [0.5, 0.866, 0],
                            [0, 0, 1]])
translation_vector = np.array([1, 2, 3])
cam_coords = np.dot(panda_coords, rotation_matrix.T) + translation_vector
"""
# TEST DATA END;




# estimate camera to panda translation/rotation matrix
affine_matrix = cv2.estimateAffine3D(panda_coords, cam_coords)[1]


print("Estimated Affine Matrix:")
print(affine_matrix)

# Reproject camera coordinates back to panda coordinates using the transformation matrix
reprojected_panda_coords = np.dot(cam_coords - affine_matrix[:3, 3], np.linalg.inv(affine_matrix[:3, :3]).T)


# Calculate the error for each axis
error = reprojected_panda_coords - panda_coords
    
mean_error = np.mean(error, axis=0)
median_error = np.median(error, axis=0)

print("\nReprojection Result:")
print("Mean Error:", mean_error)
print("Median Error:", median_error)

print("\nSaving matrix to ./calibration_matrix.csv")

affine_matrix.tofile('calibration_matrix.csv',sep=',',format='%10.16f')