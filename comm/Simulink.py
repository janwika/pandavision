import matlab.engine
from time import sleep


class Simulink:
    
    def __init__(self, engine, session):
        self.session = session    
        self.m = matlab.engine.connect_matlab(engine)        
            
    def getAttribute(self, attr, valName):
        return self.m.get_param(f'{self.session}/{attr}', valName)
    
    def setAttribute(self, attr, valName, val):
        try:
            self.m.set_param(f'{self.session}/{attr}', valName, val, nargout=200)
        except:
            pass
            
    def getRealTimeValue(self, port):
        return self.m.eval(f"rto.InputPort({port}).Data")

    def sendCenter(self, image, control, target, angleAttr):
        center = image.get_center()
        angle = image.get_angle()
        sleep(1)
        self.setAttribute(target, "Value", f"{image.convert_to_simulink_notation(center)}")
        sleep(1)
        center_floor = center
        center_floor[0][2] = 0
        self.setAttribute("TARGET_FLOOR", "Value", f"{image.convert_to_simulink_notation(center_floor)}")
        sleep(1)
        self.setAttribute(angleAttr, "Value", f"{angle}")
        sleep(1)
        self.setAttribute("START_GRIP", "Value", "1")
        sleep(5)
        self.setAttribute("START_GRIP", "Value", "0")
