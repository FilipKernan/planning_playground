# -*- coding: utf-8 -*-
from setuptools import setup

packages = [
    "planning_playground",
    "planning_playground.map",
    "planning_playground.motion_models",
    "planning_playground.search",
    "planning_playground.smoothers",
    "planning_playground.test",
    "planning_playground.viz",
]

package_data = {"": ["*"]}

install_requires = [
    "cython>=3.0.11,<4.0.0",
    "matplotlib>=3.9.0,<4.0.0",
    "networkx>=3.3,<4.0",
    "numpy>=2.0.0,<3.0.0",
    "opencv-python>=4.10,<5.0",
    "pyqt6>=6.7.0,<7.0.0",
    "pytest>=8.2.2,<9.0.0",
    "rsplan>=1.0.10,<2.0.0",
    "scipy>=1.13.1,<2.0.0",
    "setuptools>=74.0.0,<75.0.0",
    "shapely>=2.0.4,<3.0.0",
    "triangle>=20230923,<20230924",
]

setup_kwargs = {
    "name": "planning-playground",
    "version": "0.2.3",
    "description": "This is a planning playground for learning motion and task planning",
    "long_description": "# planning_playground\nThis is a standalone playground to learn and play with motion and task planning algorithms\n\n\nCurrently just working on a base holonomic A* implementation with operates on a 2d map with a state space of (x, y, theta)\n",
    "author": "Filip Kernan",
    "author_email": "filip.kernan@gmail.com",
    "maintainer": "None",
    "maintainer_email": "None",
    "url": "None",
    "packages": packages,
    "package_data": package_data,
    "install_requires": install_requires,
    "python_requires": ">=3.11,<4.0",
}
from planning_playground.build import *

build(setup_kwargs)

setup(**setup_kwargs)
