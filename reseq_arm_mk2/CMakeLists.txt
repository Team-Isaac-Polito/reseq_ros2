cmake_minimum_required(VERSION 3.5)
project(reseq_arm_mk2)

find_package(ament_cmake REQUIRED)

ament_package()

foreach(dir config launch meshes urdf viz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)
