import os
import re
import sys
import subprocess
           
#python script that automatically find the ros packages'dependencies and install them

def solve_dep(pkg):
    #create package directory name
    if '_' in pkg:
        pkg_dir = pkg.split('_')
        pkg_dir = '-'.join(pkg_dir)
    else:
        pkg_dir = pkg
    deps = f"{pkg}"
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
        print(stdout.strip())
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
                    deps += f"""\
                                {dep}"""
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
                    print(f"Dependency added: {dep}")
                    deps_set.add(dep)
            if (not found):
                #error unrelated to missing depedendencies
                #print(stderr.strip())
                missing_deps = False   
        else:
            missing_deps = False

def main():
    #ros packages passed by command line
    for i in range(1, len(sys.argv)):
        solve_dep(sys.argv[i])

if __name__ == "__main__":
    main()
