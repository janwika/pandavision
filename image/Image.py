import open3d as o3d
import numpy as np
import itertools
import copy
import matplotlib.pyplot as plt

class Image:
    
    def __init__(self, sensor, planeConf):
        self.plane = None
        self.inliers = None
        self.intersection = None
        self.mask = None
        self.center = None
        self.planeConf = planeConf
        
        #   read images
        self.color_raw = sensor.get_rgb()
        self.depth_raw = sensor.get_depth()
        
        #   create RGBD
        self.rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_raw, self.depth_raw)
        
        #   create pointcloud
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            self.rgbd,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
        )

        #   flip transformation (revert  flip caused by camera)
        self.pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
    def estimate_mask(self, mask):
        #   set all depth  pixels other than the mask's
        depth = np.asarray(self.depth_raw)
        mat = np.full((depth.shape[0], depth.shape[1]), -1).astype(np.uint16)
        matProj = np.full((depth.shape[0], depth.shape[1]), -1).astype(np.uint16)

        for p in mask:
            x = int(p[1])
            y = int(p[0])
            mat[x][y] = depth[x][y]
            matProj[x][y] = depth[x][y] - 100
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_raw, o3d.geometry.Image((mat).astype(np.uint16)))
        rgbdProj = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_raw, o3d.geometry.Image((matProj).astype(np.uint16)))
        
        #   pointcloud of projeccted mask voxels
        self.mask = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
            project_valid_depth_only = True
        )
        self.mask.paint_uniform_color([1.0, 0, 0])
        
        #   pointcloud of projeccted mask voxels offset in depth by 100
        projectionMask = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbdProj,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
            project_valid_depth_only = True
        )
        projectionMask.paint_uniform_color([0, 0, 1.0])
        
        self.mask.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        projectionMask.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        intersections = []
        
        #   calculate intersections with supporting plane
        for (p1, p2) in zip(self.mask.points, projectionMask.points):
            intersection = self.line_intersection_with_plane(p1, p2)
            if(intersection == None): continue
            intersections.append(intersection)
            
        intersections = np.array(intersections)
        
        self.intersection = o3d.geometry.PointCloud()
        self.intersection.points = o3d.utility.Vector3dVector(intersections)
        self.intersection.paint_uniform_color([0, 0, 1.0])
        
        self.center = np.array([self.intersection.get_center()])
        
    def line_intersection_with_plane(self, p1, p2):
        # Parametric equation of the line
        def line_eq(t):
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            z = p1[2] + t * (p2[2] - p1[2])
            return [x, y, z]

        # Calculate the 't' value
        denominator = self.plane[0] * (p2[0] - p1[0]) + self.plane[1] * (p2[1] - p1[1]) + self.plane[2] * (p2[2] - p1[2])
        if denominator == 0:
            return None  # The line is parallel to the plane

        t = -(self.plane[0] * p1[0] + self.plane[1] * p1[1] + self.plane[2] * p1[2] + self.plane[3]) / denominator

        return line_eq(t)
        
    def estimate_plane(self):       
        points = np.array(self.pcd.points)
        #   cuts off points beloow threshold to avoid detecting the floor
        floor = points[:, 2] < self.planeConf['FLOOR_CUTOFF']
        segmented_pcd = copy.deepcopy(self.pcd)
        segmented_pcd.points = o3d.utility.Vector3dVector(points[floor == 0])
        
        plane_model, inliers = segmented_pcd.segment_plane(distance_threshold=self.planeConf['RANSAC_THRESHOLD'],
                                         ransac_n=self.planeConf['RANSAC_N'],
                                         num_iterations=self.planeConf['RANSAC_ITERATIONS'])
        [a, b, c, d] = plane_model
        self.plane = [a, b, c, d]
        
        inlier_cloud = segmented_pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([0, 1.0, 0])
        
        self.inliers = inlier_cloud
        
    def  get_pcd(self):
        return self.pcd
    
    def  get_rgbd(self):
        return self.rgbd
    
    def  get_inliers(self):
        return self.inliers
    
    def get_intersection(self):
        return self.intersection
    
    def get_center(self):
        return self.center