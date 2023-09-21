import numpy as np
from segmentation.Yolo import Yolo
from segmentation.Detectron import Detectron2
from image.Image import Image
from image.Visualizer import Visualizer
from sensor.File import File
from sensor.RealSense import RealSense
from comm.Simulink import Simulink
import yaml
from simple_term_menu import TerminalMenu

config_path = './config.YAML'

with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    planeConf = config['PLANE']
    imageConf = config['IMAGE']
    segmentConf = config['SEGMENT']
    simulinkConf = config['SIMULINK']

path = imageConf['FILE_PATH']

# get and prepare the transformation matrix obtained from Hand Eye Calibration
calibration_matrix = np.genfromtxt('calibration_matrix.csv', delimiter=',')
calibration_matrix = calibration_matrix.reshape(3,4)
calibration_matrix = np.vstack((calibration_matrix, [0, 0, 0, 1]))

print("Capture Data from Image or Intel RealSense?")
terminal_menu = TerminalMenu(["Image", "Intel RealSense"])
sensor = terminal_menu.show()

if(sensor == 0):
    name = input('Name of the Image: ')
    sensor = File(f"{path}/rgb/{name}",  f"{path}/depth/{name}")
else:
    print('Capturing Depth and Color...')
    sensor = RealSense(imageConf['WIDTH'], imageConf['HEIGHT']) 
    
sensor.capture()

print("Use YoloV8 or Detectron2 for Segmentation?")
terminal_menu = TerminalMenu(["YoloV8", "Detectron2"])
model = terminal_menu.show()



if(model == 0):
    print("Use Stock YoloV8 or Finetuned?")
    terminal_menu = TerminalMenu(["Stock", "Finetuned"])
    model_type = terminal_menu.show()
    print('Loading segmentation model...')
    if(model_type == 1):
        segment = Yolo('finetuned', segmentConf)
    else:
        segment = Yolo('stock', segmentConf)
else:
    segment = Detectron2('stock', segmentConf)

segment.load_model()

print('Inferencing...')

masks = segment.get_masks(np.array(sensor.get_rgb()))

if(masks == None):
    print('No objects detected. Exiting..')
    exit()

print("What object should be used?")
terminal_menu = TerminalMenu(masks['objects'])
object = terminal_menu.show()

print('Estimating position of objects...')

img = Image(sensor, planeConf, calibration_matrix)
vis =  Visualizer()

img.estimate_plane()
img.estimate_mask(masks['masks'][object])

print('How should the results be visualized?')
terminal_menu = TerminalMenu(["RGBD", "only PCD", "PCD with inliers of supporting plane", "PCD with detection", "Nothing"])
visualization = terminal_menu.show()

if(visualization == 0):
    vis.plot_rgbd(img)
elif(visualization == 1):
    vis.plot_pcd(img)
elif(visualization == 2):
    vis.plot_pcd_inliers(img)
elif(visualization == 3):
    vis.plot_pcd_projected_intersection(img)
    
print('Send to Panda?')
terminal_menu = TerminalMenu(["Yes", "No"])
send = terminal_menu.show()

if(send == 0):
    print('Sending...')
    simulink = Simulink(simulinkConf['ENGINE_NAME'], simulinkConf['SIMULINK_SESSION_NAME'])
    simulink.sendCenter(img, simulinkConf['CONTROL_ATTRIBUTE'])
    print('Center Point successfully!')