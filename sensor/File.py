from .Sensor import Sensor
import open3d as o3d

class File(Sensor):
    
    def __init__(self, rgb,  depth):
        self.rgb = o3d.io.read_image(rgb)
        self.depth = o3d.io.read_image(depth)
    
    def get_depth(self):
        return self.depth
    
    def get_rgb(self):
        return self.rgb