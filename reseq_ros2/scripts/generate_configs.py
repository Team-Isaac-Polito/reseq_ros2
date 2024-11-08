import os
import yaml
import sys

# Set the path to the config directory relative to the scripts directory
config_path = os.path.join(os.path.dirname(__file__), '../config')
temp_config_path = os.path.join(config_path, 'temp')

# Ensure the temp directory exists
os.makedirs(temp_config_path, exist_ok=True)

# Function to merge multiple YAML files into a single dictionary
def merge_yaml(file_paths):
    merged_data = {}
    for file_path in file_paths:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            merged_data.update(data)
    return merged_data

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

# Function to generate final YAML configuration file
def generate_final_config(include_file):
    # Initialize main configuration with the include file content
    with open(os.path.join(config_path, include_file), 'r') as file:
        main_config = yaml.safe_load(file)

    # If include key is present, merge the included files
    if 'include' in main_config:
        include_files = main_config['include']
        main_config = create_configs(main_config, include_files)
    
        # Save the final merged configuration to a temporary YAML file
        temp_filename = f"{os.path.splitext(include_file)[0]}.yaml"
        with open(os.path.join(temp_config_path, temp_filename), 'w') as outfile:
            yaml.dump(main_config, outfile, default_flow_style=False)

        # Copy the RealSense config file to the temp directory
        if 'realsense_config' in main_config:
            copy_realsense_config(main_config['realsense_config'])

# Function to generate reseq_controllers.yaml based on the number of modules and parameters from the generic file
def generate_controllers_config(generic_config_file):
    with open(os.path.join(temp_config_path, generic_config_file), 'r') as file:
        generic_config = yaml.safe_load(file)
    
    agevar_file = generic_config["include"]["agevar_consts"]
    with open(os.path.join(config_path, agevar_file), 'r') as file:
        agevar_config = yaml.safe_load(file)

    with open(os.path.join(config_path, "reseq_controllers.yaml"), 'r') as file:
        controllers_config = yaml.safe_load(file)

    num_modules = generic_config['num_modules']
    wheel_separation = agevar_config['agevar_consts']['d']
    wheel_radius = agevar_config['agevar_consts']['r_eq']

    for i in range(num_modules):
        controller_name = f"diff_controller{i + 1}"
        controllers_config['controller_manager']['ros__parameters'][controller_name] = {
            'type': 'diff_drive_controller/DiffDriveController'
        }
        controllers_config[controller_name] = {
            'ros__parameters': {
                'left_wheel_names': [f"left_front_wheel_{i + 1}_joint", f"left_back_wheel_{i + 1}_joint"],
                'right_wheel_names': [f"right_front_wheel_{i + 1}_joint", f"right_back_wheel_{i + 1}_joint"],
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'wheel_separation': wheel_separation,
                'wheel_radius': wheel_radius,
                'wheels_per_side': 2,
                'use_stamped_vel': True,
                'enable_odom_tf': True if i == 0 else False,
                'enable_odom': True if i == 0 else False
            }
        }

    with open(os.path.join(temp_config_path, 'reseq_controllers.yaml'), 'w') as outfile:
        yaml.dump(controllers_config, outfile, default_flow_style=False)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python generate_configs.py <config_file>")
    else:
        config_file = sys.argv[1]
        generate_final_config(config_file)
        # Generate reseq_controllers.yaml to avoid fatal errors
        generate_controllers_config(config_file)
