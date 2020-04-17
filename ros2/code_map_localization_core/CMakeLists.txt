cmake_minimum_required(VERSION 3.5)
project(code_map_localization_core)

# find dependencies
find_package(ament_cmake REQUIRED)

add_custom_target(libcodemap ALL
  COMMAND $(MAKE)
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

install(FILES build/libcodemap.so
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
