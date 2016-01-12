import numpy as np
from sklearn.gaussian_process import GaussianProcess
from matplotlib import pyplot as pl

np.random.seed(1)


def f(x):
    """The function to predict."""
    return x * np.sin(x)
#----------------------------------------------------------------------
# SKLEARN
#----------------------------------------------------------------------
# now the noisy case
X = np.linspace(0.1, 9.9, 20)
X = np.atleast_2d(X).T

# Observations and noise
y = f(X).ravel()
dy = 0.5 + 1.0 * np.random.random(y.shape)
noise = np.random.normal(0, dy)
y += noise

# Mesh the input space for evaluations of the real function, the prediction and
# its MSE
x = np.atleast_2d(np.linspace(0, 10, 1000)).T

# Instanciate a Gaussian Process model
gp = GaussianProcess(corr='squared_exponential', theta0=1e-1,
                     thetaL=1e-3, thetaU=1,
                     nugget=(dy / y) ** 2,
                     random_start=100)

# Fit to data using Maximum Likelihood Estimation of the parameters
gp.fit(X, y)

# Make the prediction on the meshed x-axis (ask for MSE as well)
y_pred, MSE = gp.predict(x, eval_MSE=True)
sigma = np.sqrt(MSE)

# Plot the function, the prediction and the 95% confidence interval based on
# the MSE
fig = pl.figure()
pl.plot(x, f(x), 'r:', label=u'$f(x) = x\,\sin(x)$')
pl.errorbar(X.ravel(), y, dy, fmt='r.', markersize=10, label=u'Observations')
pl.plot(x, y_pred, 'b-', label=u'Prediction')
pl.fill(np.concatenate([x, x[::-1]]),
        np.concatenate([y_pred - 1.9600 * sigma,
                       (y_pred + 1.9600 * sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')
pl.xlabel('$x$')
pl.ylabel('$f(x)$')
pl.ylim(-10, 20)
pl.legend(loc='upper left')

pl.show(block=False)


#----------------------------------------------------------------------
# GPy
#----------------------------------------------------------------------

import pylab as pb
pb.ion()
import GPy

kernel = GPy.kern.Exponential(input_dim=1, variance=1., lengthscale=1)

Y = np.atleast_2d(y).T

m = GPy.models.GPRegression(X,Y,kernel)
m.unconstrain('')               # may be used to remove the previous constrains
m.constrain_positive('.*rbf_variance')
#m.constrain_bounded('.*lengthscale',0.4,1. )
#m.constrain_fixed('.*noise',0.025)
m.optimize()

print m
m.plot()

#----------------------------------------------------------------------
# GPy Heteroscedastic
#----------------------------------------------------------------------
import numpy as np
import pylab as pb
import GPy

def f(X):
        return 10. + .1*X + 2*np.sin(X)/X
        #return 0.00 * X

fig,ax = pb.subplots()
ax.plot(np.linspace(-15,25),f(np.linspace(-10,20)),'r-')
ax.grid()

###########################
X = np.random.uniform(-10,20, 50)
X = X[~np.logical_and(X>-2,X<3)] #Remove points between -2 and 3 (just for illustration)
X = np.hstack([np.random.uniform(-1,1,1),X]) #Prepend a point between -1 and 1  (just for illustration)
error = np.random.normal(0,.02,X.size)
Y = f(X) + error
fig,ax = pb.subplots()
ax.plot(np.linspace(-15,25),f(np.linspace(-10,20)),'r-')
ax.plot(X,Y,'kx',mew=1.5)
ax.grid()

kern = GPy.kern.RBF(input_dim = 1, ARD=True)
#kern = GPy.kern.MLP(1) + GPy.kern.Bias(1)


m = GPy.models.GPHeteroscedasticRegression(X[:,None],Y[:,None],kern)
error = np.ones(X.size) * 5
error[20:39] = 0.01
#m.param_array[4:43] = abs(error) # by regular expression does not keep the order(alphabetic order instead)
m['.*het_Gauss.variance'] = abs(error)[:,None] #Set the noise parameters to the error in Y
m.het_Gauss.variance.fix() #We can fix the noise term, since we already know it
m.optimize()

m.plot_f() #Show the predictive values of the GP.
pb.errorbar(X,Y,yerr=np.array(m.likelihood.flattened_parameters).flatten(),fmt=None,ecolor='r',zorder=1)
pb.grid()
pb.plot(X,Y,'kx',mew=1.5)

def noise_effect(noise):
    m.het_Gauss.variance[:1] = noise
    m.het_Gauss.variance.fix()
    m.optimize()

    m.plot_f() 
    pb.errorbar(X.flatten(),Y.flatten(),yerr=np.array(m.likelihood.flattened_parameters).flatten(),fmt=None,ecolor='r',zorder=1)        
    pb.plot(X[1:],Y[1:],'kx',mew=1.5)
    pb.plot(X[:1],Y[:1],'ko',mew=.5)
    pb.grid()

#from IPython.html.widgets import *
#interact(noise_effect, noise=(0.1,2.))

#Heteroscedastic model
m1 = GPy.models.GPHeteroscedasticRegression(X[:,None],Y[:,None],kern)
m1.het_Gauss.variance = .05
m1.het_Gauss.variance.fix()
m1.optimize()

# Homoscedastic model
m2 = GPy.models.GPRegression(X[:,None],Y[:,None],kern)
#m2['.*Gaussian_noise'] = .05
#m2['.*noise'].fix()
m2.optimize()

m1.plot_f()
pb.title('Homoscedastic model')
m2.plot_f()
pb.title('Heteroscedastic model')

print "Kernel parameters (optimized) in the heteroscedastic model"
print m1.kern
print "\nKernel parameters (optimized) in the homoscedastic model"
print m2.kern

kern = GPy.kern.MLP(1) + GPy.kern.Bias(1)

m = GPy.models.GPHeteroscedasticRegression(X[:,None],Y[:,None],kern)
m.optimize()

fig, ax = pl.subplots(1,1,figsize=(13,5))
m.plot_f(ax=ax)
m.plot_data(ax=ax)
m.plot_errorbars_trainset(ax=ax, alpha=1)
fig.tight_layout()
pb.grid()


########### FROM PR FIXING THE LENGTH ISSUE ######
import numpy as np
import GPy

Ns = 100
#Generate data
X = 10*np.random.random((Ns,1))
y = np.sin(X) + np.random.random((Ns,1)) - 0.5
Xtest = np.linspace(np.min(X), np.max(X), 121)
Xtest = Xtest[:,None]

# Estimate model
heteroscedastic_variance = np.random.random((X.shape[0],1)) # Here you define your variances
g = GPy.likelihoods.HeteroscedasticGaussian(variance=heteroscedastic_variance,Y_metadata=np.arange(Ns)[:,None])
m = GPy.core.GP(X=X, Y=y, kernel=GPy.kern.RBF(X.shape[1]), likelihood=g)
m.likelihood.variance.fix()
m.optimize()

# Make predictions
noise_dict = {'output_index':1*np.ones_like(Xtest).astype(int)}
ytest = m.predict(Xtest, Y_metadata=noise_dict)[0]

