#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['translator_driver']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts']

setup(**d)
