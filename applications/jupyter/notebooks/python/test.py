import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit, minimize_scalar

x = [1,2,3.2,4,5]
y = [1,4,16,4,1]

def f(x, p1, p2, p3):
    return p3 * (p1 / ((x-p2)**2 + (p1/2)**2))   

# def gaussian(x, mu, sig):
#     return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

# def gauss_function(x, a, x0, sigma):
#     return a*np.exp(-(x-x0)**2/(2*sigma**2))


p0 = (8, 16, 0.1) # guess perameters 
plt.plot(x,y,"ro")
popt, pcov = curve_fit(f, x, y, p0)

# find the peak
fm = lambda x: -f(x, *popt)
r = minimize_scalar(fm, bounds=(1, 5))
print("maximum:", r["x"], f(r["x"], *popt))  #maximum: 2.99846874275 18.3928199902

x_curve = np.linspace(1, 5, 100)
plt.plot(x_curve, f(x_curve, *popt))
plt.plot(r['x'], f(r['x'], *popt), 'ko')
plt.show()