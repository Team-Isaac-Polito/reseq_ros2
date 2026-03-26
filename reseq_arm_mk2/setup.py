from glob import glob

from setuptools import find_packages, setup

package_name = 'reseq_arm_mk2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(
        include=['reseq_arm_mk2', 'reseq_arm_mk2.*', 'kdl_parser_py', 'kdl_parser_py.*']
    ),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*launch.py')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/meshes', glob('meshes/*')),
        (f'share/{package_name}/viz', glob('viz/*')),
        (f'share/{package_name}/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='URDF Description package for RESE.Q MK2 Manipulator',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_arm_controller = reseq_arm_mk2.cartesian_arm_controller:main',
            'arm_state_bridge = reseq_arm_mk2.arm_state_bridge:main',
            'coordinate_controller = reseq_arm_mk2.coordinate_controller:main',
            'moveit_controller = reseq_arm_mk2.moveit_controller:main',
        ],
    },
)
