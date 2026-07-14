cmake_minimum_required(VERSION 3.8)
project(ros_plc_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/crx_pick_place.py
    scripts/cell_io.py
    scripts/cell_mode.py
    scripts/color_classifier.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
