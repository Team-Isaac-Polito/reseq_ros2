from setuptools import find_packages
from setuptools import setup

setup(
    name='reseq_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('reseq_interfaces', 'reseq_interfaces.*')),
)
