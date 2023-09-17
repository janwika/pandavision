import matlab.engine
import yaml

config_path = './config.YAML'

with open(config_path, "r") as f:
    config = yaml.safe_load(f)
    simulinkConf = config['SIMULINK']
    
    
session = simulinkConf['SIMULINK_SESSION_NAME']    
m = matlab.engine.connect_matlab(simulinkConf['ENGINE_NAME'])

# m.get_param(f'{session}/{attr}', valName)

print(f"{session}/Display1")

m.workspace['rto'] = m.get_param(f"{session}/Display1", 'RunTimeObject')

H_IT = m.eval('rto.InputPort(1).Data')

print(H_IT)
