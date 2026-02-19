cmake_minimum_required(VERSION 3.5)
project(reseq_arm_mk2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

foreach(dir config launch meshes urdf viz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  # "reseq_arm_mk2/coordinate_controller.py"
  "reseq_arm_mk2/moveit_controller.py"
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
