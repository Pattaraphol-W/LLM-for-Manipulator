cmake_minimum_required(VERSION 3.8)
project(keyence_plc_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(keyence_plc_ethernet_driver
  PACKAGE_DIR src/keyence_plc_ethernet_driver
)

# Automatically find all Python scripts in the scripts folder
file(GLOB PYTHON_SCRIPTS "scripts/*.py")

# Install Python scripts
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
