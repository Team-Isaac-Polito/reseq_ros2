from setuptools import setup
from glob import glob

package_name = 'reseq_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team ISAAC',
    maintainer_email='team.isaac@polito.it',
    description='ROS2 code for ReseQ robot',
    license='GNU GPL v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication = reseq_ros2.communication:main',
            'agevar = reseq_ros2.agevar:main',
            'remote_test = reseq_ros2.remote_test:main',
            'enea = reseq_ros2.enea:main',
            'scaler = reseq_ros2.scaler:main',
            'realsense = reseq_ros2.realsense:main',
        ],
    },
)
