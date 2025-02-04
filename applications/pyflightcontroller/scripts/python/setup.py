from setuptools import setup
from Cython.Build import cythonize
import os

ext_modules = [
    cythonize("main.pyx", language_level="3"),
    cythonize("ibus.pyx"),
    cythonize("toolkit.pyx"),
    cythonize("esc_calibration.pyx"),
    cythonize("pico_potentiometer.pyx")
]

setup(
    ext_modules=ext_modules,
    install_requires=['RPi.GPIO'],
)


# python setup.py build_ext --inplace
# python main.py
