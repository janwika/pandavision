import matlab.engine


class Simulink:
    
    def __init__(self, engine, session):
        self.session = session    
        self.m = matlab.engine.connect_matlab(engine)
        self.m.workspace['rto'] = self.m.get_param(f"{self.session}/Display1", 'RunTimeObject')          
            
    def getAttribute(self, attr, valName):
        return self.m.get_param(f'{self.session}/{attr}', valName)
    
    def setAttribute(self, attr, valName, val):
        try:
            self.m.set_param(f'{self.session}/{attr}', valName, val, nargout=200)
        except:
            pass
            
    def getRealTimeValue(self, attr, port):
    	return self.m.eval(f"rto.InputPort({port}).Data")
          
    def sendCenter(self, image, attr):
        center = image.get_center()
        self.setAttribute(attr, f"{center}")
