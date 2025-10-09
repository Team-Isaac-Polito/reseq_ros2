import os
import shutil
import subprocess
import sys

import yaml
import argparse

# Set the path to the config directory relative to the scripts directory
config_path = os.path.join(os.path.dirname(__file__), '../config')
temp_config_path = os.path.join(config_path, 'temp')

# Ensure the temp directory exists
os.makedirs(temp_config_path, exist_ok=True)


# Function to clear the temp directory
def clear_temp_directory(temp_path):
    if os.path.exists(temp_path):
        shutil.rmtree(temp_path)
        os.makedirs(temp_path, exist_ok=True)


# Function to include and merge additional YAML files specified in the 'include' section
def create_configs(main_config, include_files):
    for key, file in include_files.items():
        file_path = os.path.join(config_path, file)
        with open(file_path, 'r') as f:
            included_config = yaml.safe_load(f)
            main_config.setdefault(key, {}).update(included_config[key])
    return main_config


# Function to copy the RealSense config file to the temp directory
def copy_realsense_config(realsense_config):
    src = os.path.join(config_path, realsense_config)
    dst = os.path.join(temp_config_path, realsense_config)
    if os.path.exists(src):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(src, 'r') as f_src, open(dst, 'w') as f_dst:
            f_dst.write(f_src.read())


# Function to find the '/dev/video' device corresponding to each usb camera
def find_video_devices(num_cam):
    try:
        video_dev_output = subprocess.check_output(['v4l2-ctl', '--list-devices']).decode()
    except Exception as e:
        print(f'Error in finding USB cameras: {e}')
        return []
    if 'UC60 Video' not in video_dev_output:
        print('No camera device was found!')
        return []
    video_dev_output = video_dev_output.split('\n')
    dev_found = 0
    video_dev = []
    for i in range(0, len(video_dev_output)):
        if video_dev_output[i].startswith('UC60 Video'):
            video_dev.append(video_dev_output[i + 1].strip('\t'))
            dev_found += 1
        if dev_found == num_cam:
            break
    return video_dev


# Function to copy the i-th usb camera config file to the temp directory
def copy_usb_camera_config(usb_camera_config, devices, i):
    src = os.path.join(config_path, usb_camera_config)
    dst = os.path.join(temp_config_path, f'usb_camera_config_{i}')
    if os.path.exists(src):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(src, 'r') as f_src, open(dst, 'w') as f_dst:
            f_src_yaml = yaml.safe_load(f_src)
            f_src_yaml['/**']['ros__parameters']['video_device'] = devices[i]
            yaml.dump(f_src_yaml, f_dst)


# Function to generate final YAML configuration file
def generate_final_config(include_file):
    # Initialize main configuration with the include file content
    with open(os.path.join(config_path, include_file), 'r') as file:
        main_config = yaml.safe_load(file)

    # If include key is present, merge the included files
    if 'include' in main_config:
        include_files = main_config['include']
        main_config = create_configs(main_config, include_files)

        # Copy the RealSense config file to the temp directory
        if 'realsense_config' in main_config:
            copy_realsense_config(main_config['realsense_config'])

        # Genearate a config file for each usb camera, then copy all of them in the temp directory
        if 'usb_camera_config' in main_config:
            cameras = True
            for sensor in main_config['sensors']:
                name = list(sensor.keys())[0]
                if name == 'usb_cameras':
                    if sensor[name]:
                        num_usb_cam = sensor[name]
                        devices = find_video_devices(num_usb_cam)
                        if len(devices) < num_usb_cam:
                            print(f'{num_usb_cam - len(devices)} cameras were not found!')
                        if devices:
                            for i in range(0, len(devices)):
                                copy_usb_camera_config(
                                    main_config['usb_camera_config'], devices, i
                                )
                        else:
                            cameras = False

            # If cameras are not present, then set usb cameras to False in the final config file
            if not cameras:
                for sensor in main_config['sensors']:
                    name = list(sensor.keys())[0]
                    if name == 'usb_cameras':
                        sensor[name] = False

        # Save the final merged configuration to a temporary YAML file
        temp_filename = f'{os.path.splitext(include_file)[0]}.yaml'
        with open(os.path.join(temp_config_path, temp_filename), 'w') as outfile:
            yaml.dump(main_config, outfile, default_flow_style=False)


# Function to generate reseq_controllers.yaml based on the number of modules and parameters
# from the generic file
def generate_controllers_config(generic_config_file, use_sim_time: bool):
    with open(os.path.join(temp_config_path, generic_config_file), 'r') as file:
        generic_config = yaml.safe_load(file)

    agevar_file = generic_config['include']['agevar_consts']
    with open(os.path.join(config_path, agevar_file), 'r') as file:
        agevar_config = yaml.safe_load(file)

    with open(os.path.join(config_path, 'reseq_controllers.yaml'), 'r') as file:
        controllers_config = yaml.safe_load(file)

    num_modules = generic_config['num_modules']
    wheel_separation = agevar_config['agevar_consts']['d']
    wheel_radius = agevar_config['agevar_consts']['r_eq']

    controllers_config['controller_manager']['ros__parameters']['use_sim_time'] = use_sim_time

    for i in range(num_modules):
        controller_name = f'diff_controller{i + 1}'
        controllers_config['controller_manager']['ros__parameters'][controller_name] = {
            'type': 'diff_drive_controller/DiffDriveController'
        }
        controllers_config[controller_name] = {
            'ros__parameters': {
                'left_wheel_names': [
                    f'mod{i + 1}__left_front_wheel',
                    f'mod{i + 1}__left_back_wheel',
                ],
                'right_wheel_names': [
                    f'mod{i + 1}__right_front_wheel',
                    f'mod{i + 1}__right_back_wheel',
                ],
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'wheel_separation': wheel_separation,
                'wheel_radius': wheel_radius,
                'wheels_per_side': 2,
                'use_stamped_vel': True,
                'enable_odom_tf': True if i == 0 else False,
                'enable_odom': True if i == 0 else False,
            }
        }

    with open(os.path.join(temp_config_path, 'reseq_controllers.yaml'), 'w') as outfile:
        yaml.dump(controllers_config, outfile, default_flow_style=False)


if __name__ == '__main__':
    # Using argparse to tidily manage args from command line
    parser = argparse.ArgumentParser(
        usage='Usage: python generate_configs.py <config_file>',
        description='Use this python file to generate configurations in the /config/temp folder.'
    )
    parser.add_argument(
        'config_file',
        help='Configuration file name',
    )
    parser.add_argument(
        '--use_sim_time',
        '-s',
        action='store_true',
        required=False,
        default=False,
        help='Set this to true if you want all nodes described\
            in the configuration files to use the Simulation Time',
    )

    args = parser.parse_args()
    config_file = args.config_file
    use_sim_time = args.use_sim_time

    try:
        # Clear the temp directory before generating new configs
        clear_temp_directory(temp_config_path)
        generate_final_config(config_file)
        # Generate reseq_controllers.yaml to avoid fatal errors
        generate_controllers_config(config_file, use_sim_time)
        sys.exit(0)
    except Exception as e:
        print(f'Error generating configuration files: {e}')
        sys.exit(1)