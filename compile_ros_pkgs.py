import logging
import re
import subprocess
import sys

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('RPKG Solver')


def install_dependencies(pkgs):
    """
    Resolves dependencies for a given list of ROS packages.

    Args:
        pkgs (list): A list of ROS package names.

    The function performs the following steps:
    1. Creates a directory name based on the first package name.
    2. Constructs a command to source the ROS entry point, create necessary directories,
       generate a .rosinstall file, import the packages, install dependencies, and build the packages.
    3. Executes the command in a subprocess.
    4. If the command fails due to missing dependencies, it extracts the missing dependency from the error message,
       adds it to the list of dependencies, and retries the command.
    5. Repeats the process until all dependencies are resolved or an unrelated error occurs.

    Prints:
        - Standard output and error messages from the subprocess.
        - Added dependencies during the resolution process.

    Exits:
        - If an unrelated error occurs during the subprocess execution.
    """
    pkg_dir = pkgs[0].replace('_', '-')
    deps = ' '.join(pkgs)
    cmd_template = f"""source /ros_entrypoint.sh && \
            mkdir -p /{pkg_dir}/src && \
            cd /{pkg_dir} && \
            rosinstall_generator --rosdistro $ROS_DISTRO \
                {{deps}} \
            > {pkg_dir}.rosinstall && \
            vcs import src < {pkg_dir}.rosinstall && \
            rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -t buildtool -t build -t build_export -t buildtool_export -t exec --skip-keys="$SKIP_KEYS" """  # noqa
    deps_set = set()
    missing_deps = True
    while missing_deps:
        cmd = cmd_template.format(deps=deps)
        process = subprocess.Popen(
            cmd,
            text=True,
            shell=True,
            stderr=subprocess.PIPE,
            executable='/bin/bash',
        )
        _, stderr = process.communicate()
        if process.returncode != 0:
            dep_re = re.compile(
                r'No definition of \[([^"]+)\] for OS version|Cannot locate rosdep definition for \[([^"]+)\]'  # noqa
            )
            found = False
            for line in stderr.splitlines():
                match = dep_re.search(str(line))
                if match:
                    dep = match.group(1) or match.group(2)
                    if dep not in deps_set:
                        found = True
                        deps += f' {dep}'
                        deps_set.add(dep)
                        logger.info(f'Dependency added: {dep}')
            if not found:
                logger.error(stderr.strip())
                missing_deps = False
                exit(1)
        else:
            missing_deps = False
    logger.info(f'All dependencies: {", ".join(deps_set)}')
    return pkg_dir


def build_and_cleanup(pkg_dir):
    """
    Builds the ROS packages using colcon and cleans up the src, build, and log directories.

    Args:
        pkg_dir (str): The directory name based on the first package name.
    """
    cmd = f"""source /ros_entrypoint.sh && \
            cd /{pkg_dir} && \
            colcon build --event-handlers=console_direct+ --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
            sed -i "\$i ros_source_env /{pkg_dir}/install/local_setup.bash \n" /ros_entrypoint.sh && \
            rm -rf src build log"""  # noqa
    process = subprocess.Popen(
        cmd,
        text=True,
        shell=True,
        executable='/bin/bash',
    )
    process.communicate()
    if process.returncode != 0:
        exit(1)


def main():
    pkgs = sys.argv[1:]
    pkg_dir = install_dependencies(pkgs)
    build_and_cleanup(pkg_dir)


if __name__ == '__main__':
    main()
