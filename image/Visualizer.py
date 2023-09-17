import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class Visualizer:
    
    def plot_pcd(self, image):
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([image.get_pcd(), coordinate_frame])
        
    def plot_rgbd(self,  image):
        plt.subplot(1, 2, 1)
        plt.title('Greyscale image')
        plt.imshow(image.get_rgbd().color)
        plt.subplot(1, 2, 2)
        plt.title('Depth image')
        plt.imshow(image.get_rgbd().depth)
        plt.show()
        
    def plot_pcd_inliers(self, image):
        inliers = image.get_inliers()
        if(inliers == None): raise Exception('No Plane was estimated.')
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([image.get_pcd(), inliers, coordinate_frame])
        
    def plot_pcd_projected_intersection(self,  image):
        intersection = image.get_intersection()
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([image.get_pcd(), intersection, image.get_bounding_box(), coordinate_frame])
