from ultralytics import YOLO
import json
from .Segment import Segment

class Yolo(Segment):
    
    def __init__(self, modelType, segmentConf):
        super().__init__(modelType, segmentConf)
        with open('./finetuned_classes.json', 'r') as file:
            self.classes = json.load(file)
    
    def load_model(self):
        if(self.modelType == 'finetuned'):
            self.model = YOLO('./finetuned_pre_own_labels.pt')
        else:
            self.model = YOLO('yolov8x-seg.pt')
    
    def get_masks(self, img):
        
        if(self.model == None): raise Exception('Model not loaded')
        results = self.model.predict(img, save=self.segmentConf['SAVE_MASK'])
        if(results[0].masks == None): return None
        
        print(self.classes)
        
        if(self.modelType == 'stock'):
            objects = [self.model.names[int(c)] for c in results[0].boxes.cls]
        else:
            objects = [self.classes[int(c)] for c in results[0].boxes.cls]
            
        return {'objects': objects, 'masks': results[0].masks.xy}
            