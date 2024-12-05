
import random
import numpy as np
from scipy.optimize import curve_fit, minimize_scalar

start = 0
stop = 9
step = 1

def recalibrate(start, stop, step):
    """ Recalibration example with stubs """

    input = np.arange(start, stop, step)
    size = input.shape[0]
    ion_responses = np.zeros(size, dtype=np.int16)

    for x in range(size):
        move_mirror_to_position(input[x])
        ion_responses[x] = measure_ion_response()

    return pick_best_pos(ion_responses)

def move_mirror_to_position(x):
    """ Stubb for serial connection """
    pass

def measure_ion_response():
    """ Stub for ion responses """
    return random.randint(0, 100)

def gaussian(x, a, x0, sigma):
    """ Gaussian function used for fit """
    return a * np.exp(-(x - x0)**2/(2 * sigma**2))

def pick_best_pos(ion_responses):
    """ Curve fit using Scipy implementation """

    # X values of responses
    x = list(range(len(ion_responses)))

    # Curve fit gaussian to responses
    # popt are the optimal fit values
    popt, pcov = curve_fit(gaussian, x, ion_responses)

    # Find minimum of fit negative gaussian function
    fm = lambda x: -gaussian(x, *popt)
    r = minimize_scalar(fm)

    return r["x"]

def test_pick_best_pos():
    """ Test fit function """

    # Mock data
    mock_responses = np.array([0, 0, 1, 2, 20, 20, 2, 1, 0, 0])

    # Test fit function
    pos = pick_best_pos(mock_responses)
    assert pos == 4.500000015697534, f"expected 4.5, got: {pos}"

test_pick_best_pos()