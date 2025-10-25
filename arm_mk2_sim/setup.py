from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_mk2_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        (f'share/{package_name}/worlds', glob('worlds/*.world')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team_isaac',
    maintainer_email='TODO@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
