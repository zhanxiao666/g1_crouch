cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

option(ENABLE_ARM_IK_LCM "Enable LCM bridge for Python right-arm IK target override" ON)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  /opt/unitree_robotics/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
)

if(ENABLE_ARM_IK_LCM)
  find_path(LCM_INCLUDE_DIR lcm/lcm-cpp.hpp
    PATHS
      /usr/include
      /usr/local/include
      $ENV{CONDA_PREFIX}/include
      $ENV{HOME}/miniconda3/include
      $ENV{HOME}/miniconda3/lib/python3.13/site-packages/include
  )
  find_library(LCM_LIBRARY lcm
    PATHS
      /usr/lib
      /usr/local/lib
      /usr/lib/x86_64-linux-gnu
      $ENV{CONDA_PREFIX}/lib
      $ENV{HOME}/miniconda3/lib
      $ENV{HOME}/miniconda3/lib/python3.13/site-packages/lib
  )
  if(NOT LCM_INCLUDE_DIR OR NOT LCM_LIBRARY)
    message(FATAL_ERROR "ENABLE_ARM_IK_LCM=ON requires LCM headers and library")
  endif()
  include_directories(${LCM_INCLUDE_DIR})
  add_compile_definitions(UNITREE_RL_ENABLE_ARM_IK_LCM)
endif()

file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/*.cpp)

add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  cnpy_lib
  ZLIB::ZLIB
)

if(ENABLE_ARM_IK_LCM)
  link_libraries(${LCM_LIBRARY})
endif()

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)
