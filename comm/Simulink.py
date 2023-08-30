import matlab.engine


class Simulink:
    
    def __init__(self, engine, session):
        self.session = session    
        self.m = matlab.engine.connect_matlab(engine)            
            
    def getAttribute(self, attr):
        return self.m.get_param(f'{self.session}/{attr}', 'Value')
    
    def setAttribute(self, attr, val):
        try:
            self.m.set_param(f'{self.session}/{attr}', 'Value', val, nargout=200)
        except:
            pass
        
    def sendCenter(self, image, attr):
        center = image.get_center()
        self.setAttribute(attr, f"{center}")