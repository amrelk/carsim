import numpy as np
import pacejka

'''
' x: [0: car x pos [m]; 1: car y pos [m/s]; 2: car x speed [m/s]; 3: car y speed [m/s];
'     4: car theta [rad]; 5: car omega [rad/s];
'     6: FL speed [rad/s]; 7: FR speed [rad/s]; 8: RL speed [rad/s]; 9: RR speed [rad/s]]
' y: [ground z force [N]; drag x force [N]; car x accel [m/s^2]; current [A]]
' u: [desired current [A]]
'''

def updfn(t, x, u, params):
  xdot = np.zeros(10)
  motor_speed = (x[8] + x[9])/2 * params['trans']['ratio'] # rad/s
  I = min(min((params['motor']['Vin'] - (motor_speed / params['motor']['Kv'])) / params['motor']['R'], params['motor']['Imax']), u[0]) # Arms
  T_axle = params['motor']['Kt'] * I * params['trans']['ratio']
  a = 0 # slip angle, will be deleted when turning is added
  SR_FL = 0 if x[2] == 0 else np.clip(((x[6] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_FR = 0 if x[2] == 0 else np.clip(((x[7] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_RL = 0 if x[2] == 0 else np.clip(((x[8] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_RR = 0 if x[2] == 0 else np.clip(((x[9] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  Tfx_FL = pacejka.longitudinal(SR_FL, params['vehicle']['mass']*9.81/4, params['tire']['pacejka']) * params['tire']['radius']
  Tfx_FR = pacejka.longitudinal(SR_FR, params['vehicle']['mass']*9.81/4, params['tire']['pacejka']) * params['tire']['radius']
  Tfx_RL = pacejka.longitudinal(SR_RL, params['vehicle']['mass']*9.81/4, params['tire']['pacejka']) * params['tire']['radius']
  Tfx_RR = pacejka.longitudinal(SR_RR, params['vehicle']['mass']*9.81/4, params['tire']['pacejka']) * params['tire']['radius']
  xdot[6] = (-Tfx_FL) / (params['tire']['MOI'])
  xdot[7] = (-Tfx_FR) / (params['tire']['MOI'])
  xdot[8] = (T_axle - Tfx_RL) / (params['tire']['MOI'] + params['trans']['MOI']/2 + params['motor']['MOI']/2)
  xdot[9] = (T_axle - Tfx_RR) / (params['tire']['MOI'] + params['trans']['MOI']/2 + params['motor']['MOI']/2)
  xdot[0] = x[2]
  xdot[2] = (Tfx_RL / params['tire']['radius'] + Tfx_RR / params['tire']['radius']) / params['vehicle']['mass']
  return xdot
  
def outfn(t, x, u, params):
  a = 0
  SR_FL = 0 if x[2] == 0 else np.clip(((x[6] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_FR = 0 if x[2] == 0 else np.clip(((x[7] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_RL = 0 if x[2] == 0 else np.clip(((x[8] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  SR_RR = 0 if x[2] == 0 else np.clip(((x[9] * params['tire']['radius']) / (x[2] * np.cos(a))) - 1, -10, 10)
  return [SR_FL, SR_FR, SR_RL, SR_RR]