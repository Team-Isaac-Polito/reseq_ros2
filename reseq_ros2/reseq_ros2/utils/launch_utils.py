import yaml
from ament_index_python.packages import get_package_share_directory
from yaml import SafeLoader

# Default config file path
share_folder = get_package_share_directory('reseq_ros2')
config_path = f'{share_folder}/config/temp'
default_filename = 'reseq_mk2_can.yaml'


def parse_config(filename):
    with open(filename) as f:
        return yaml.load(f, Loader=SafeLoader)


def get_addresses(config):
    addresses = []
    for mod in config['modules']:
        addresses.append(mod['address'])
    return addresses


def get_joints(config):
    joints = []
    for mod in config['modules']:
        if mod['hasJoint']:
            joints.append(mod['address'])
    return joints


def get_end_effector(config):
    for mod in config['modules']:
        if mod['hasEndEffector']:
            return mod['address']
    return 0
