import pyOpt
from pyOpt import pySLSQP

def objfunc(x):

  f = x[0]**2 + x[1]**2 + x[2]**2
  g = []

  fail = 0
  return f,g, fail


opt_prob = pyOpt.Optimization('TP37 Constrained Problem',objfunc)
opt_prob.addObj('f')
opt_prob.addVar('x1','c',lower=0.0,upper=42.0,value=10.0)
opt_prob.addVar('x2','c',lower=0.0,upper=42.0,value=10.0)
opt_prob.addVar('x3','c',lower=0.0,upper=42.0,value=10.0)

print opt_prob

slsqp = pyOpt.pySLSQP.SLSQP()

slsqp.setOption('IPRINT', -1)

[fstr, xstr, inform] = slsqp(opt_prob,sens_type='FD')

print opt_prob.solution(0)