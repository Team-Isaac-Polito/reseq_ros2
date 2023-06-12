from launch import LaunchDescription
from launch_ros.actions import Node

import sys
from yaml import SafeLoader
import yaml

def extract_argv(arg_name):
    args = [s for s in sys.argv if arg_name in s]
    
    if len(args)<1:
        return
    
    filepath = args[0][(len(arg_name)+2):]
    return filepath


def parse_config(path):
    with open(path) as f:
        return yaml.load(f, Loader=SafeLoader)
    

def parse_arguments():
    config_file = extract_argv("config_file")
    simulation = extract_argv("simulation") or False
    
    if (simulation!=False):
        simulation = simulation.lower()=='true'

    if(not config_file):
        raise Exception("No configuration file passed!")

    config = parse_config(config_file)
    return config, simulation

def get_addresses(config):
    out = []
    for mod in config['modules']:
        out.append(mod['address'])
    return out


def get_joints(config):
    out = []
    for mod in config['modules']:
        if mod['hasJoint']:
            out.append(mod['address'])
    return out


def generate_launch_description():
    config, simulation = parse_arguments()

    ld = LaunchDescription()

    agevar_node = Node(
        package="reseq_ros2",
        executable="agevar",
        parameters= [
            {'a': config['dimensions']['a']},
            {'b': config['dimensions']['b']},
            {'d': config['dimensions']['d']},
            {'r_eq': config['dimensions']['r_eq']},
            {'modules': get_addresses(config)},
            {'joints': get_joints(config)},
        ]
    )
    ld.add_action(agevar_node)

    if not simulation:
        communication_node = Node(
            package="reseq_ros2",
            executable="communication",
            parameters=[
                {'can_channel': config['canbus']['channel']},
                {'modules': get_addresses(config)},
                {'joints': get_joints(config)},
            ]
        )
        ld.add_action(communication_node)

    return ld
