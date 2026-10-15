cmake_minimum_required(VERSION 3.19)
project(robot_state_estimation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robot_state_estimation)

# A symlink install keeps source permissions; generate an executable launcher
# for the module whose source is intentionally a normal Python package file.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/formal_odometry_node.py
  "#!/usr/bin/env python3\nfrom robot_state_estimation.formal_odometry_node import main\n\nif __name__ == '__main__':\n    main()\n")
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/formal_odometry_node.py
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

install(PROGRAMS
  robot_state_estimation/imu_adapter_node.py
  robot_state_estimation/wheel_odometry_node.py
  ${CMAKE_CURRENT_BINARY_DIR}/formal_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_state_estimation_primitives
    test/test_state_estimation_primitives.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_formal_freshness test/test_formal_freshness.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
