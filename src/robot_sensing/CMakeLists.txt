cmake_minimum_required(VERSION 3.19)
project(robot_sensing)

find_package(ament_cmake REQUIRED)

configure_file(cmake/scan_normalizer_launcher.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/scan_normalizer.py @ONLY)
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/scan_normalizer.py
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/scan_normalizer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/scan_normalizer.py DESTINATION share/${PROJECT_NAME}/runtime)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scan_normalizer
    ../../tests/interface/test_scan_normalizer.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
