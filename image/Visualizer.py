import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class Visualizer:
    
    def plot_pcd(self, image):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.add_geometry(image.get_pcd())
        o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.8)
        vis.run()
        
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
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.add_geometry(image.get_pcd())
        vis.add_geometry(inliers)
        vis.run()
        
    def plot_pcd_projected_intersection(self,  image):
        intersection = image.get_intersection()
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.add_geometry(image.get_pcd())
        vis.add_geometry(intersection)
        vis.add_geometry(image.get_bounding_box())        
        vis.run()