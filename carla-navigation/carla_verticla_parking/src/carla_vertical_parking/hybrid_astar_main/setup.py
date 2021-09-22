from setuptools import setup
import sys

assert sys.version_info.major == 3 and sys.version_info.minor >= 6, \
    "The hybrid astar repo is designed to work with Python 3.7 and greater." \
    + "Please install it before proceeding."

setup(
    name='hybrid_astar',
    py_modules=['hybrid_astar'],
    version= '1.0',
    install_requires=[
        'matplotlib',
        'numpy',
        'scipy',
    ],
    description="hybrid astar algorithm",
    author="Han Ruihua",
)