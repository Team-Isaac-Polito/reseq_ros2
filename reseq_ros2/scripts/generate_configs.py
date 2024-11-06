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

# Function to generate final YAML configuration file
def generate_final_config(include_file):
    # Initialize main configuration with the include file content
    with open(os.path.join(config_path, include_file), 'r') as file:
        main_config = yaml.safe_load(file)

    # If include key is present, merge the included files
    if 'include' in main_config:
        include_files = main_config['include']
        main_config = create_configs(main_config, include_files)
        del main_config['include']  # Clean up the include key after merging
    
        # Save the final merged configuration to a temporary YAML file
        temp_filename = f"{os.path.splitext(include_file)[0]}.yaml"
        with open(os.path.join(temp_config_path, temp_filename), 'w') as outfile:
            yaml.dump(main_config, outfile, default_flow_style=False)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python generate_configs.py <config_file>")
    else:
        config_file = sys.argv[1]
        generate_final_config(config_file)
        # Generate reseq_controllers.yaml to avoid fatal errors
        generate_final_config("reseq_controllers.yaml")
