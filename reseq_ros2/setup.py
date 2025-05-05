from glob import glob

from setuptools import setup

package_name = 'reseq_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/scripts', glob('scripts/*.py')),
        (f'share/{package_name}/description', glob('description/*.xacro')),
        (f'share/{package_name}/description/macros', glob('description/macros/*.xacro')),
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
            'communication = reseq_ros2.communication:main',
            'agevar = reseq_ros2.agevar:main',
            'remote_test = reseq_ros2.remote_test:main',
            'enea = reseq_ros2.enea:main',
            'scaler = reseq_ros2.scaler:main',
            'joint_publisher = reseq_ros2.joint_publisher:main',
            'feedback_replicator = reseq_ros2.feedback_replicator:main',
            'emulator_remote_controller = reseq_ros2.emulator_remote_controller:main',
            'detection_data_handler = reseq_ros2.detection_data_handler:main',
        ],
    },
)
