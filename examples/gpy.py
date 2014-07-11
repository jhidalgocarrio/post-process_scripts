import pylab as pb
pb.ion()
import numpy as np
import GPy

# 1-dimensional model
X = np.random.uniform(-3.,3.,(20,1))
Y = np.sin(X) + np.random.randn(20,1)*0.05

#Square exponential kernel
kernel = GPy.kern.rbf(input_dim=1, variance=1., lengthscale=1.)

m = GPy.models.GPRegression(X,Y,kernel)

print m
m.plot()

m.ensure_default_constraints() # or similarly m.constrain_positive('')

m.optimize()

# 2-dimensional model for sample inputs and outputs
X = np.random.uniform(-3.,3.,(50,2))
Y = np.sin(X[:,0:1]) * np.sin(X[:,1:2])+np.random.randn(50,1)*0.05

# define kernel
ker = GPy.kern.Matern52(2,ARD=True) + GPy.kern.white(2)

# create simple GP model
m = GPy.models.GPRegression(X,Y,ker)

# contrain all parameters to be positive
m.constrain_positive('')

# optimize and plot
m.optimize('tnc', max_f_eval = 1000)
m.plot()
print(m)
