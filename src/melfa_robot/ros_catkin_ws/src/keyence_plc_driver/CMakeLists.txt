cmake_minimum_required(VERSION 2.8.3)
project(keyence_plc_driver)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

install(PROGRAMS 
  scripts/lrtb2000c_driver.py
  scripts/ps4_reader.py
  scripts/joint_state.py
  scripts?joint_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(FILES package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

