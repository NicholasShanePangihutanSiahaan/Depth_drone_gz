cmake_minimum_required(VERSION 3.8)
project(uav_plantation_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(livox_top_config test/test_livox_top.py)
endif()

ament_package()
