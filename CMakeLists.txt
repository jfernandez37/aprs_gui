cmake_minimum_required(VERSION 3.8)
project(aprs_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclpy
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/gui_node.py
  nodes/controllers.py
  nodes/image_test_node.py
  nodes/simple_gui_node.py
  nodes/old_gui_node.py
  nodes/publish_dummy_trays.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
