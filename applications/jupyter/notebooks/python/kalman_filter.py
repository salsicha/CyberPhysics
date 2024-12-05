

# Initialization
# 1. Initialize the state of the filter
# 2. Initialize our belief in the state


# Predict
# 1. Use process model to predict state at the next time step
# 2. Adjust belief to account for the uncertainty in prediction    


# Update
# 1. Get a measurement and associated belief about its accuracy
# 2. Compute residual between estimated state and measurement
# 3. Compute scaling factor based on whether the measurement
# or prediction is more accurate
# 4. set state between the prediction and measurement based 
# on scaling factor
# 5. update belief in the state based on how certain we are 
# in the measurement


# Predict
# 𝐱,𝐏 are the state mean and covariance. They correspond to 𝑥 and 𝜎2
# 𝐅 is the state transition function. When multiplied by 𝐱 it computes the prior
# 𝐐 is the process covariance. It corresponds to 𝜎2𝑓𝑥
# 𝐁 and 𝐮 are new to us. They let us model control inputs to the system

# Update
# 𝐇 is the measurement function. We haven't seen this yet in this book and I'll explain it later. If you mentally remove 𝐇 from the equations, you should be able to see these equations are similar as well.
# 𝐳,𝐑 are the measurement mean and noise covariance. They correspond to 𝑧 and 𝜎2𝑧 in the univariate filter (I've substituted 𝜇 with 𝑥 for the univariate equations to make the notation as similar as possible).
# 𝐲 and 𝐊 are the residual and Kalman gain

import numpy as np
from scipy.linalg import expm, block_diag
import copy

x = np.array([[10.0], [4.5]])
P = np.diag([500., 49.])

dt = 0.1
F = np.array([[1, dt], [0, 1]])

Q = Q_discrete_white_noise(dim=2, dt=1., var=2.35)

B = 0.  # no input control
u = 0
x, P = predict(x, P, F, Q, B, u)


H = np.array([[1., 0.]])
R = np.array([[5.]])
z = 1.
x, P = update(x, P, z, R, H)


def predict(self, u=None, B=None, F=None, Q=None):
    """
    Predict next state (prior) using the Kalman filter state propagation
    equations.
    Parameters
    ----------
    u : np.array, default 0
        Optional control vector.
    B : np.array(dim_x, dim_u), or None
        Optional control transition matrix; a value of None
        will cause the filter to use `self.B`.
    F : np.array(dim_x, dim_x), or None
        Optional state transition matrix; a value of None
        will cause the filter to use `self.F`.
    Q : np.array(dim_x, dim_x), scalar, or None
        Optional process noise matrix; a value of None will cause the
        filter to use `self.Q`.
    """

    if B is None:
        B = self.B
    if F is None:
        F = self.F
    if Q is None:
        Q = self.Q
    elif np.isscalar(Q):
        Q = np.eye(self.dim_x) * Q


    # x = Fx + Bu
    if B is not None and u is not None:
        self.x = np.dot(F, self.x) + np.dot(B, u)
    else:
        self.x = np.dot(F, self.x)

    # P = FPF' + Q
    self.P = self._alpha_sq * np.dot(np.dot(F, self.P), F.T) + Q

    # save prior
    self.x_prior = self.x.copy()
    self.P_prior = self.P.copy()


def update(self, z, R=None, H=None):
    """
    Add a new measurement (z) to the Kalman filter.
    If z is None, nothing is computed. However, x_post and P_post are
    updated with the prior (x_prior, P_prior), and self.z is set to None.
    Parameters
    ----------
    z : (dim_z, 1): array_like
        measurement for this update. z can be a scalar if dim_z is 1,
        otherwise it must be convertible to a column vector.
        If you pass in a value of H, z must be a column vector the
        of the correct size.
    R : np.array, scalar, or None
        Optionally provide R to override the measurement noise for this
        one call, otherwise  self.R will be used.
    H : np.array, or None
        Optionally provide H to override the measurement function for this
        one call, otherwise self.H will be used.
    """

    # set to None to force recompute
    self._log_likelihood = None
    self._likelihood = None
    self._mahalanobis = None

    if z is None:
        self.z = np.array([[None]*self.dim_z]).T
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()
        self.y = np.zeros((self.dim_z, 1))
        return

    if R is None:
        R = self.R
    elif np.isscalar(R):
        R = eye(self.dim_z) * R

    if H is None:
        z = reshape_z(z, self.dim_z, self.x.ndim)
        H = self.H

    # y = z - Hx
    # error (residual) between measurement and prediction
    self.y = z - np.dot(H, self.x)

    # common subexpression for speed
    PHT = np.dot(self.P, H.T)

    # S = HPH' + R
    # project system uncertainty into measurement space
    self.S = np.dot(H, PHT) + R
    self.SI = self.inv(self.S)
    # K = PH'inv(S)
    # map system uncertainty into kalman gain
    self.K = np.dot(PHT, self.SI)

    # x = x + Ky
    # predict new x with residual scaled by the kalman gain
    self.x = self.x + dot(self.K, self.y)

    # P = (I-KH)P(I-KH)' + KRK'
    # This is more numerically stable
    # and works for non-optimal K vs the equation
    # P = (I-KH)P usually seen in the literature.

    I_KH = self._I - np.dot(self.K, H)
    self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(self.K, R), self.K.T)

    # save measurement and posterior state
    self.z = copy.deepcopy(z)
    self.x_post = self.x.copy()
    self.P_post = self.P.copy()





def order_by_derivative(Q, dim, block_size):
    """
    Given a matrix Q, ordered assuming state space
        [x y z x' y' z' x'' y'' z''...]
    return a reordered matrix assuming an ordering of
       [ x x' x'' y y' y'' z z' y'']
    This works for any covariance matrix or state transition function
    Parameters
    ----------
    Q : np.array, square
        The matrix to reorder
    dim : int >= 1
       number of independent state variables. 3 for x, y, z
    block_size : int >= 0
        Size of derivatives. Second derivative would be a block size of 3
        (x, x', x'')
    """

    N = dim * block_size

    D = np.zeros((N, N))

    Q = np.array(Q)
    for i, x in enumerate(Q.ravel()):
        f = np.eye(block_size) * x

        ix, iy = (i // dim) * block_size, (i % dim) * block_size
        D[ix:ix+block_size, iy:iy+block_size] = f

    return D


def Q_discrete_white_noise(dim, dt=1., var=1., block_size=1, order_by_dim=True):
    """
    Returns the Q matrix for the Discrete Constant White Noise
    Model. dim may be either 2, 3, or 4 dt is the time step, and sigma
    is the variance in the noise.
    Q is computed as the G * G^T * variance, where G is the process noise per
    time step. In other words, G = [[.5dt^2][dt]]^T for the constant velocity
    model.
    Parameters
    -----------
    dim : int (2, 3, or 4)
        dimension for Q, where the final dimension is (dim x dim)
    dt : float, default=1.0
        time step in whatever units your filter is using for time. i.e. the
        amount of time between innovations
    var : float, default=1.0
        variance in the noise
    block_size : int >= 1
        If your state variable contains more than one dimension, such as
        a 3d constant velocity model [x x' y y' z z']^T, then Q must be
        a block diagonal matrix.
    order_by_dim : bool, default=True
        Defines ordering of variables in the state vector. `True` orders
        by keeping all derivatives of each dimensions)
        [x x' x'' y y' y'']
        whereas `False` interleaves the dimensions
        [x y z x' y' z' x'' y'' z'']
    Examples
    --------
    >>> # constant velocity model in a 3D world with a 10 Hz update rate
    >>> Q_discrete_white_noise(2, dt=0.1, var=1., block_size=3)
    array([[0.000025, 0.0005  , 0.      , 0.      , 0.      , 0.      ],
           [0.0005  , 0.01    , 0.      , 0.      , 0.      , 0.      ],
           [0.      , 0.      , 0.000025, 0.0005  , 0.      , 0.      ],
           [0.      , 0.      , 0.0005  , 0.01    , 0.      , 0.      ],
           [0.      , 0.      , 0.      , 0.      , 0.000025, 0.0005  ],
           [0.      , 0.      , 0.      , 0.      , 0.0005  , 0.01    ]])
    References
    ----------
    Bar-Shalom. "Estimation with Applications To Tracking and Navigation".
    John Wiley & Sons, 2001. Page 274.
    """

    if dim not in [2, 3, 4]:
        raise ValueError("dim must be between 2 and 4")

    if dim == 2:
        Q = [[.25*dt**4, .5*dt**3],
             [ .5*dt**3,    dt**2]]
    elif dim == 3:
        Q = [[.25*dt**4, .5*dt**3, .5*dt**2],
             [ .5*dt**3,    dt**2,       dt],
             [ .5*dt**2,       dt,        1]]
    else:
        Q = [[(dt**6)/36, (dt**5)/12, (dt**4)/6, (dt**3)/6],
             [(dt**5)/12, (dt**4)/4,  (dt**3)/2, (dt**2)/2],
             [(dt**4)/6,  (dt**3)/2,   dt**2,     dt],
             [(dt**3)/6,  (dt**2)/2 ,  dt,        1.]]

    if order_by_dim:
        return block_diag(*[Q]*block_size) * var
    return order_by_derivative(np.array(Q), dim, block_size) * var



def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z (shape {}) must be convertible to shape ({}, 1)'.format(z.shape, dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z