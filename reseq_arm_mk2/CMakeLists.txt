cmake_minimum_required(VERSION 3.5)
project(reseq_arm_mk2)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclpy REQUIRED)

set(PYTHON_INSTALL_DIR ${Python3_SITELIB})

foreach(dir config launch meshes urdf viz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

install(DIRECTORY reseq_arm_mk2/
  DESTINATION ${PYTHON_INSTALL_DIR}/reseq_arm_mk2
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY kdl_parser_py
  DESTINATION ${PYTHON_INSTALL_DIR}
)

install(PROGRAMS
  "reseq_arm_mk2/cartesian_arm_controller.py"
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  "reseq_arm_mk2/coordinate_controller.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME coordinate_controller
)

install(PROGRAMS
  "reseq_arm_mk2/moveit_controller.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME moveit_controller
)

ament_package()
