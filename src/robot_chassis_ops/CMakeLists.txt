cmake_minimum_required(VERSION 3.19)
project(robot_chassis_ops)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_chassis_ops)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/chassis_ops_node
  "#!/usr/bin/env python3\nfrom robot_chassis_ops.chassis_ops_node import main\n\nif __name__ == '__main__':\n    main()\n")
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/chassis_ops_node
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/chassis_ops_node DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ops_node test/test_ops_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
