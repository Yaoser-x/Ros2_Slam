cmake_minimum_required(VERSION 3.19)
project(robot_navigation_guard)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_navigation_guard)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/navigation_guard_node
  "#!/usr/bin/env python3\nfrom robot_navigation_guard.navigation_guard_node import main\n\nif __name__ == '__main__':\n    main()\n")
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/navigation_guard_node
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/navigation_guard_node DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_guard test/test_guard.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_guard_node test/test_guard_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_adapter_ros_executor
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/integration/test_ros_adapter_executor.py
    TIMEOUT 100 APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
