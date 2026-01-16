cmake_minimum_required(VERSION 3.5)
project(ecn_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

ament_auto_add_library(ecn_common src/color_detector.cpp)
target_include_directories(ecn_common PUBLIC include)

install(TARGETS ecn_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

ament_export_dependencies(cv_bridge)
ament_export_libraries(ecn_common)
ament_export_include_directories(include)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
