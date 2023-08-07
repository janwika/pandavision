import pyrealsense2 as rs
import open3d as o3d
import numpy as np
from .Sensor import Sensor

class RealSense(Sensor):
    
    def __init__(self, width, height):
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        self.rgb = None
        self.depth = None
        
    def capture(self):
        self.pipe.start(self.cfg)
        
        # wait for camera to adjust
        for x in range(20):
            self.pipe.wait_for_frames()
            
        frameset = self.pipe.wait_for_frames()

        rgb = np.asanyarray(frameset.get_color_frame().get_data())
        depth = np.asanyarray(frameset.get_depth_frame().get_data())

        self.rgb = o3d.geometry.Image((rgb))
        self.depth = o3d.geometry.Image((depth))
        
        self.pipe.stop()
        
    
    def get_depth(self):
        return self.depth
    
    def get_rgb(self):
        return self.rgb