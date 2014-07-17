'''
Created on Jul 17, 2014

@author: posilva
'''
from distutils.core import setup

setup(
    name='mav2rosgenerator',
    version='0.1.0',
    author='Pedro Marques da Silva',
    author_email='posilva@gmail.com',
    packages=['mav2ros', 'mav2ros.test'],
    scripts=[],
    url='http://pypi.python.org/pypi/TowelStuff/',
    license='LICENSE.txt',
    description='A Mavlink protocol to ROS API Generator.',
    long_description=open('README.txt').read(),
    install_requires=[],
)