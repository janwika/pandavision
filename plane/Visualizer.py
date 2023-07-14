import numpy as np
import open3d as o3d
from PIL import Image
import matplotlib.pyplot as plt

class Visualizer:
    
    def plotImage(self, rgb, depth):
        color_raw = o3d.io.read_image(rgb)
        depth_raw = o3d.io.read_image(depth)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_raw, depth_raw)
        print(rgbd_image)
        
        plt.subplot(1, 2, 1)
        plt.title('Grayscale image')
        plt.imshow(rgbd_image.color)
        plt.subplot(1, 2, 2)
        plt.title('Depth image')
        plt.imshow(rgbd_image.depth)
        #plt.show()
        
        
        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
        )

        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        downpcd = pcd.voxel_down_sample(voxel_size=0.5)
        
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.8)
        vis.run()

        
        
path = '/mnt/c/Users/janwi/Desktop/Programmieren/Bachelorarbeit/pandavision/test_data'

vis = Visualizer()
pic = 7
vis.plotImage(f"{path}/cuttlery/rgb/0000{pic}.png", f"{path}/cuttlery/depth/0000{pic}.png")