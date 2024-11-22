
# A lighthouse is somewhere off a piece of straight coastline at a position x along the coast 
# and a distance y out to sea. It emits a series of short, 
# highly collimated flashes at random intervals and hence random azimuths. 
# These pulse are intercepted on the coast by photo-detectors that record only the fact that a flash has occurred, 
# but not the azimuth from which it came. N
# N flashes so far have been recorded at positions {ai}. Where is the lighthouse?

# https://mjoldfield.com/atelier/2017/10/gulls-lighthouse.html
# https://github.com/mjoldfield/gulls-lighthouse

# P(h|e) = P(e|h) * P(h) / P(e)
# h = hypothesis
# e = evidence
# P(h) = prior probability of h
# P(e|h) = likelihood of observing e given h
# P(e) = normalization factor

# x = horizontal position of lighthouse
# y = vertical position of lighthouse
# a = position on shore of beam

# P(x,y|a) = P(a|x,y) * P(x,y) / P(a)

# Distribution/likelihood of a single beam on shore is given by Cauchy distribution
# P(a|x,y) = (1/pi)(y/[y^2 + (a - x)^2])

# Multiple beams:
# P({a_of_i_flashes}|x,y) = Product_over_i( [1/pi] * [y / (y^2 + (a_i - x)^2)] )

# P(x,y):
# Uniform prior
# for -l < x < l
# and 0 < y < d
# P(x,y) = 1 / (2d * l)
# e.g. if l=10meters and d=10meters
# then the probability of the lighthouse being at x=1, y=1 is:
# 1 / 200
# Why 2d? not d???

# P(x,y|a) = Product_over_i_to_n([y / (y^2 + (a_i - x)^2)]) / (pi^n * [2d * l] * Product_over_i(P(a_i)))

# Z absorbs all constants

# P(x,y|a) = (1 / Z) * Product_i_to_n(y / [y^2 + (a_i - x)^2])

# Pseudo code:
# a = [a1, a2, a3, ...] # random numbers, between -pi/2, pi/2? Binning?
# x = [x1, x2, x3, ...] # 0 to 10, 10cm binning?
# y = [y1, y2, y3, ...] # 0 to 10, 10cm binning?
# prod = None
# for i, a_i in enumerate(a):
#   if i == 0:
#       prod = y / [y^2 + (a_i - x)^2]
#   else:
#       prod *= y / [y^2 + (a_i - x)^2]

# Finally, plot heatmap after some iterations...


import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import cauchy

# TODO: set probability of a occurring in each bin of a:
# a_list = []
# x = 50.0
# y = 45.0
# for i in range(100):
#     a_bin = float(i)
#     val = (1.0 / np.pi) * (y / (y**2 + (a_bin - x)**2))
#     a_list.append(val)
# a_arr = np.asarray(a_list)


# fig, ax = plt.subplots(1, 1)
# x = np.linspace(cauchy.ppf(0.01), cauchy.ppf(0.99), 100)
# print("x: ", x)
# print("x: ", x.shape)
# ccy = cauchy.pdf(x)
# print("ccy: ", ccy)
# ax.plot(x, ccy, 'r-', lw=5, alpha=0.6, label='cauchy pdf')
# plt.show()


ccy = cauchy.rvs(size=100)
print("ccy: ", ccy)
hist, bin_edges = np.histogram(ccy)
plt.hist(hist)
plt.show()

# a_index = np.random.randint(0, 100, 100)
# a = a_arr[a_index]
# print("a_arr: ", a_arr)
# print("a: ", a.shape)

# TODO: "a" needs to sample from couchy distribution around light source
a = np.random.randint(0, 100, 100)

x = np.arange(1.0, 101, dtype=np.float)
y = np.arange(1.0, 101, dtype=np.float)

xv, yv = np.meshgrid(x, y)

print("a: ", a.shape)
print("x: ", x.shape)
print("y: ", y.shape)
print("xv: ", xv.shape)
print("yv: ", yv.shape)

prod = None
for i, a_i in enumerate(a):
    if i == 0:
        prod = yv / (yv**2 + (a_i - xv)**2)
    else:
        prod *= yv / (yv**2 + (a_i - xv)**2)

print("prod: ", prod.shape)

plt.imshow(prod)
plt.show()
