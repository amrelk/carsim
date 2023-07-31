import numpy as np

def longitudinal(s, Fz, model):
  s = s * 100 # convert slip to percent
  C = model['b0']
  D = Fz * (model['b1'] * Fz + model['b2'])
  BCD = (model['b3'] * Fz**2 + model['b4'] * Fz) * np.e**(-model['b5']*Fz)
  B = BCD / (C * D)
  H = model['b9'] * Fz + model['b10']
  V = model['b11'] * Fz + model['b12']
  E = (model['b6'] * Fz**2 + model['b7'] * Fz + model['b8']) * (1 - model['b13'] * np.sign(s + H))
  Bx1 = B * (s + H)
  F = D * np.sin( C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V
  return F