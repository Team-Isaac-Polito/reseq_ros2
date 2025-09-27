from glob import glob

from setuptools import setup

package_name = 'reseq_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.utils', f'{package_name}.reseq_cv'],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/scripts', glob('scripts/*.py')),
        (f'share/{package_name}/description', glob('description/*.xacro')),
        (f'share/{package_name}/description/macros', glob('description/macros/*.xacro')),
        (f'share/{package_name}/ml-ckpt', glob('ml-ckpt/*.pt')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
            'agevar = reseq_ros2.agevar:main',
            'scaler = reseq_ros2.scaler:main',
            'emulator_remote_controller = reseq_ros2.emulator_remote_controller:main',
            'detector = reseq_ros2.detector:main',
            'pivot_controller = reseq_ros2.pivot_controller:main',
        ],
    },
)
