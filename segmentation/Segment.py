class Segment:
    def __init__(self, modelType, segmentConf):
        self.modelType = modelType
        self.segmentConf = segmentConf
        self.model = None
        
    def load_model(self):
        pass
        
    def get_masks(self, img):
        pass