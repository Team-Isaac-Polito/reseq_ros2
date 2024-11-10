import yaml
from yaml import SafeLoader
from ament_index_python.packages import get_package_share_directory

#Default config file path
share_folder = get_package_share_directory("reseq_ros2")
config_path = f'{share_folder}/config'
default_filename = "reseq_mk1_can.yaml"

def parse_config(filename):
    with open(filename) as f:
        return yaml.load(f, Loader=SafeLoader)
    
def get_addresses(config):
    l = []
    for mod in config['modules']:
        l.append(mod['address'])
    return l

def get_joints(config):
    l = []
    for mod in config['modules']:
        if mod['hasJoint']:
            l.append(mod['address'])
    return l

def get_end_effector(config):
    for mod in config['modules']:
        if mod['hasEndEffector']:
            return mod['address']