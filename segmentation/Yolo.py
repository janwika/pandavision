from ultralytics import YOLO
from .Segment import Segment

class Yolo(Segment):
    
    def load_model(self, path='yolov8x-seg_FT.pt'):
        if(self.modelType == 'stock'):
            self.model = YOLO('yolov8x-seg.pt')
        elif(self.modelType == 'finetuned'):
            self.model = YOLO(path)
    
    def get_masks(self, img):
        if(self.model == None): raise Exception('Model not loaded')
        results = self.model.predict(img, save=True)
        if(results[0].masks == None): return None
        
        objects = [self.model.names[int(c)] for c in results[0].boxes.cls]
        
        return {'objects': objects, 'masks': results[0].masks}
            