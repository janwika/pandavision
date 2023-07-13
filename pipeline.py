from segmentation.Yolo import Yolo

path = '/mnt/c/Users/janwi/Desktop/Programmieren/Bachelorarbeit/pandavision/test_data'

print('Loading segmentation model...')

segment = Yolo('stock')
segment.load_model()

print('Inferencing...')

masks = segment.get_masks([f"{path}/cuttlery/rgb/00009.png"])
print(masks['objects'])
