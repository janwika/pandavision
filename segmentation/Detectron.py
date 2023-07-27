import detectron2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
import cv2
import os
import json
from .Segment import Segment

class Detectron2(Segment):
    
    def load_model(self):
        self.cfg = get_cfg()
        # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
        # Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        self.model = DefaultPredictor(self.cfg)
        with open('./detectron_classes.json', 'r') as file:
            self.classes = json.load(file)
        
    def get_masks(self, img):
        if(self.model == None): raise Exception('Model not loaded')
        outputs = self.model(img)

        objects = []
        masks = []
        for predClass in outputs["instances"].pred_classes:
            objects.append(self.classes[predClass])
        for mask in outputs["instances"].pred_masks:
            mask = mask.cpu()
            mask = mask.numpy().astype('uint8')
            contour, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            contour = contour[0]
            masks.append([cont[0] for cont in contour])
        
        v = Visualizer(img[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
        v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        
        if(self.segmentConf['SAVE_MASK']):
            run = ''
            
            items = os.listdir('./runs/segment')
            folders = [item for item in items if os.path.isdir(os.path.join('./runs/segment', item))]
            if(len(folders) > 0):
                run = f'{len(folders) + 1}'
            
            os.makedirs(f'./runs/segment/predict{run}')
            v.save(f'./runs/segment/predict{run}/image0.jpg')
        
        return {'objects': objects, 'masks': masks}