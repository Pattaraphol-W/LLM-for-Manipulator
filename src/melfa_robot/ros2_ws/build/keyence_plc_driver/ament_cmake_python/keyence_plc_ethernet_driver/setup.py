from setuptools import find_packages
from setuptools import setup

setup(
    name='keyence_plc_ethernet_driver',
    version='0.1.0',
    packages=find_packages(
        include=('keyence_plc_ethernet_driver', 'keyence_plc_ethernet_driver.*')),
)
