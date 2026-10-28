from glob import glob
from setuptools import find_packages, setup

setup(
    name='reseq_traction_autonomy',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/reseq_traction_autonomy']),
        ('share/reseq_traction_autonomy', ['package.xml']),
        ('share/reseq_traction_autonomy/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'mcp': ['mcp>=1.12,<2'], 'lightnav': ['websockets>=12,<16']},
    entry_points={
        'console_scripts': [
            'traction_manager = reseq_traction_autonomy.manager:main',
            'traction_mcp = reseq_traction_autonomy.mcp_server:main',
            'traction_tool = reseq_traction_autonomy.client:main',
        ]
    },
)
