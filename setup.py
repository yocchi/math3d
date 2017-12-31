# -*- coding: utf-8 -*-

# Learn more: https://github.com/kennethreitz/setup.py

from setuptools import setup, find_packages


with open('README.rst') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='math3d',
    version='0.1.0',
    description='library for 3-dimensional mathematics',
    long_description=readme,
    author='yocchi',
    author_email='yocchiman@gmail.com',
    install_requires=['numpy'],
    url='https://github.com/yocchi/math3d',
    license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)
