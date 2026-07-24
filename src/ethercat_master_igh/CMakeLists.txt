cmake_minimum_required(VERSION 3.8)
project(ethercat_master_igh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_library(ETHERCAT_LIB ethercat PATHS /opt/etherlab/lib /usr/lib /usr/local/lib)
if(NOT ETHERCAT_LIB)
  message(FATAL_ERROR "libethercat not found. Install IgH EtherCAT Master userspace (e.g. /opt/etherlab).")
endif()

set(ETHERCAT_INCLUDES /opt/etherlab/include)
if(DEFINED ENV{ETHERLAB_ROOT} AND NOT "$ENV{ETHERLAB_ROOT}" STREQUAL "")
  set(ETHERCAT_INCLUDES "$ENV{ETHERLAB_ROOT}/include")
  find_library(ETHERCAT_LIB ethercat PATHS "$ENV{ETHERLAB_ROOT}/lib" REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/master/igh/ethercat_sync.cpp
  src/master/igh/ethercat_servo.cpp
  src/master/igh/igh_master_runtime.cpp
  src/motor/motor_kinematics.cpp
  src/motor/motor_profile.cpp
  src/diagnostics/runtime_logger.cpp
  src/master_api.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ETHERCAT_BACKEND_IGH=1)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERCAT_INCLUDES}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ETHERCAT_LIB}
  pthread
)

get_filename_component(ETHERCAT_LIB_DIR "${ETHERCAT_LIB}" DIRECTORY)
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${ETHERCAT_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include ${ETHERCAT_INCLUDES})
ament_export_libraries(${PROJECT_NAME})

ament_package()
