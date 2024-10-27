import os
import re
import sys
import subprocess
           
#python script that automatically find the ros packages'dependencies and install them

def solve_dep(pkgs):
    """
    Resolves dependencies for a given list of ROS packages and builds them using colcon.

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
    #create package directory name
    if '_' in pkgs[0]:
        pkg_dir = pkgs[0].split('_')
        pkg_dir = '-'.join(pkg_dir)
    else:
        pkg_dir = pkgs[0]
    deps = " ".join(pkgs)
    cmd = f"""source /ros_entrypoint.sh && \
            mkdir -p /{pkg_dir}/src && \
            cd /{pkg_dir} && \
            rosinstall_generator --rosdistro ${{ROS_DISTRO}} \
                {deps} \
            > {pkg_dir}.rosinstall && \
            vcs import src < {pkg_dir}.rosinstall && \
            rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --skip-keys="$SKIP_KEYS" && \
            colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
            sed -i "\$i ros_source_env /{pkg_dir}/install/local_setup.bash \n" /ros_entrypoint.sh"""
    deps_set = set()
    missing_deps = True
    while (missing_deps):
        #run command
        process = subprocess.Popen(cmd, text=True, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, executable="/bin/bash")
        stdout, stderr = process.communicate()
        print(stdout.strip(), flush=True)
        found = False
        if process.returncode != 0:
            #find the dependency in the error using regular expressions
            dep_re1 = re.compile(f"No definition of \[([^\"]+)\] for OS version")
            dep_re2 = re.compile(f"Cannot locate rosdep definition for \[([^\"]+)\]")
            for line in stderr.splitlines():
                match1 = dep_re1.search(str(line))
                match2 = dep_re2.search(str(line))
                if ((match1 and match1 not in deps_set) or (match2 and match2 not in deps_set)):
                    found = True
                    if match1:
                        dep = match1.group(1)
                    elif match2:
                        dep = match2.group(1)
                    deps += f" {dep}"
                    cmd = f"""source /ros_entrypoint.sh && \
                            mkdir -p /{pkg_dir}/src && \
                            cd /{pkg_dir} && \
                            rosinstall_generator --rosdistro ${{ROS_DISTRO}} \
                                {deps} \
                            > {pkg_dir}.rosinstall && \
                            vcs import src < {pkg_dir}.rosinstall && \
                            rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --skip-keys="$SKIP_KEYS" && \
                            colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
                            sed -i "\$i ros_source_env /{pkg_dir}/install/local_setup.bash \n" /ros_entrypoint.sh"""
                    print(f"Dependency added: {dep}", flush=True)
                    deps_set.add(dep)
            if (not found):
                #error unrelated to missing depedendencies
                print(stderr.strip(), flush=True)
                missing_deps = False   
                exit(1)
        else:
            missing_deps = False

def main():
    #ros packages passed by command line
    solve_dep(sys.argv[1:])

if __name__ == "__main__":
    main()
