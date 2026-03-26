cmake_minimum_required(VERSION 3.8)
project(adore_dynamics VERSION 0.0.1 LANGUAGES CXX)

# -------------------------------------------------------------------
# C++ configuration
# -------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(OpenDrive REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_map REQUIRED)

# -------------------------------------------------------------------
# Library
# -------------------------------------------------------------------
# Library sources
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.c")

# Define library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Include paths
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    adore_math::adore_math
    adore_map::adore_map
)

# -------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Library under test, used by test/CMakeLists.txt
  set(ADORE_TEST_LIB_TARGET ${PROJECT_NAME})

  add_subdirectory(test)
endif()

# -------------------------------------------------------------------
# Install & export
# -------------------------------------------------------------------
# Export target to make find_package(adore_dynamics) work in other packages
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the CMake config for downstream consumers
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ament exports for ROS 2 packages
ament_export_include_directories(include)
ament_export_dependencies(adore_math adore_map)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
