import os
import subprocess
from time import time

import psutil
import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node


def check_interface_status(interface_name):
    """Check if the specified network interface is up and running."""
    # Run the `ip` command to get interface details
    result = subprocess.run(['ip', 'link', 'show', interface_name], capture_output=True, text=True)

    # Check if the command was successful
    if result.returncode != 0:
        return False, f'Failed to get interface status. Error: {result.stderr}'

    # Check if the interface is up
    if 'UP,LOWER_UP' in result.stdout:
        return True, f'Interface {interface_name} is up and running.'
    elif f'Device "{interface_name}" does not exist' in result.stdout:
        return False, f'Interface {interface_name} does not exist.'
    else:
        return False, f'Interface {interface_name} is not up.'


def simulate_launch_test(argv):
    """
    Simulate the launch of the ROS2 test using the provided configuration file.

    This function checks if the script is being run by `launch_test`. If not,
    it launches the ROS2 test using the specified configuration file.
    """
    if 'launch_test' in os.path.basename(argv):
        return None
    else:
        cmd = [
            'ros2',
            'launch',
            'reseq_ros2',
            'reseq_launch.py',
            'config_file:=reseq_mk1_vcan.yaml',
        ]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        return process


def tear_down_process(process):
    """Terminate the process and its child processes."""
    if process is None:
        pass
    else:
        try:
            # Ensure all child processes are terminated
            parent_pid = process.pid
            parent = psutil.Process(parent_pid)
            for child in parent.children(recursive=True):
                child.terminate()

            _, still_alive = psutil.wait_procs(parent.children(), timeout=5)
            for p in still_alive:
                p.kill()  # Force kill if still alive
        except psutil.NoSuchProcess:
            pass  # The process has already exited


def extract_value(parameter_value):
    type_mapping = {
        1: parameter_value.bool_value,
        2: parameter_value.integer_value,
        3: parameter_value.double_value,
        4: parameter_value.string_value,
        5: list(parameter_value.byte_array_value),
        6: list(parameter_value.bool_array_value),
        7: list(parameter_value.integer_array_value),
        8: list(parameter_value.double_array_value),
        9: list(parameter_value.string_array_value),
    }
    return type_mapping.get(parameter_value.type)


def check_parameters(node: Node, service_name: str, expected_params: dict, timeout_sec: float):
    """Checks if the parameters of a node match the expected values."""
    # Create a client to get parameters of the node
    parameter_client = node.create_client(GetParameters, service_name)
    client_found = parameter_client.wait_for_service(timeout_sec=timeout_sec)
    if not client_found:
        return False, f'Parameter service {service_name} not available within the timeout period.'

    for param_name, expected_value in expected_params.items():
        request = GetParameters.Request()
        request.names = [param_name]
        future = parameter_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()

        # Verify the response
        if response is None:
            return False, f'Failed to get parameter {param_name} from {service_name}'

        param_value = response.values[0]
        if extract_value(param_value) != expected_value:
            return False, f'{param_name} parameter value mismatch'

    return True, ''


def check_missing_parameters(
    node: Node, service_name: str, expected_params: dict, timeout_sec: float
):
    """Checks if there are any unexpected or missing parameters on a node."""
    # Create a client to list all parameters of the node
    list_parameters_client = node.create_client(ListParameters, service_name)
    client_found = list_parameters_client.wait_for_service(timeout_sec=timeout_sec)
    if not client_found:
        return (
            False,
            f'List parameters service {service_name} not available within the timeout period.',
        )

    request = ListParameters.Request()
    future = list_parameters_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    # Verify the response
    if response is None:
        return False, f'Failed to list parameters from {service_name}'

    # all params except the 'use_sim_time' parameter
    all_parameters = response.result.names[1:]
    unexpected_parameters = set(all_parameters) - set(expected_params.keys())
    missing_parameters = set(expected_params.keys()) - set(all_parameters)

    if unexpected_parameters:
        return False, f'Unexpected parameters found: {unexpected_parameters}'
    if missing_parameters:
        return False, f'Missing parameters found: {missing_parameters}'

    return True, ''


def check_node_up(node: Node, expected_node: str, timeout_sec: float):
    """Checks if a node is up and running."""
    # Wait for the node to be up and running
    start_time = time()
    node_found = False
    while time() - start_time < timeout_sec and not node_found:
        node_names = node.get_node_names()
        node_found = expected_node in node_names
        if not node_found:
            rclpy.spin_once(node, timeout_sec=1)

    # Validate the state of the node
    if not node_found:
        return False, f'{expected_node} node was not found running within the timeout period.'
    return True, ''
