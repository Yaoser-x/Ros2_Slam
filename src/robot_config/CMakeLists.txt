cmake_minimum_required(VERSION 3.19)
project(robot_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robot_config)

foreach(entrypoint compile_config compatibility_node)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${entrypoint}
    "#!/usr/bin/env python3\nfrom robot_config.${entrypoint} import main\n\nif __name__ == '__main__':\n    raise SystemExit(main())\n")
  file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/${entrypoint}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${entrypoint} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_compatibility test/test_compatibility.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
