################################### CONFIG ###################################
# Set minimum required version
cmake_minimum_required(VERSION 3.1)
project(smv2_drive)

message(STATUS "--------------------------------------")
message(STATUS "Building: ${PROJECT_NAME}-${VERSION_STRING}")
message(STATUS "--------------------------------------")

# Set C++ Version 17
set(CMAKE_CXX_STANDARD 17)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Use new policies introduced up to this version
cmake_policy(VERSION 2.8.12)
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif(POLICY CMP0042)

# add path where cmake files must be searched for
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Include path points to the base source dir
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})

# Make sure we are under Unix
if(NOT UNIX)
    message(FATAL_ERROR "This compiles only under UNIX, sorry!(haven't tested other systems)")
endif(NOT UNIX)

# Add all the CMake modules we need
include(${CMAKE_SOURCE_DIR}/cmake/DownloadProject.cmake)

################################ PROJECT SETUP ################################
# Download SimpleMotionV2 library from github
download_project(PROJ     SimpleMotionV2
  GIT_REPOSITORY      https://github.com/GraniteDevices/SimpleMotionV2.git
  GIT_TAG             43561feffff5c03990a063a74c9c38f3183efe6a
  UPDATE_DISCONNECTED 1
)

# Include SimpleMotionV2
include_directories(${SimpleMotionV2_SOURCE_DIR}
${SimpleMotionV2_SOURCE_DIR}/utils)
############# LOGURU #############

# Download LOGuru library from github
download_project(PROJ     loguru
  GIT_REPOSITORY      https://github.com/emilk/loguru.git
  GIT_TAG             master
  UPDATE_DISCONNECTED 1
)

# Use streams
add_definitions(-DLOGURU_WITH_STREAMS)

# Setup Include paths
include_directories(${loguru_SOURCE_DIR})

############# pybind 11 #############

# Download PyBind library from github
download_project(PROJ     pybind11
  GIT_REPOSITORY      https://github.com/pybind/pybind11.git
  GIT_TAG             master
  UPDATE_DISCONNECTED 1
)

add_subdirectory(${pybind11_SOURCE_DIR})

################################### PROJECT ###################################

############# Other deps #############
# Prefer PThread
set(THREADS_PREFER_PTHREAD_FLAG ON)

# dependency
find_package(Threads REQUIRED)

############# Headers #############
file (GLOB H_FILES
  src/drive_controller.h
  src/utility/*.h
)

############# Sources #############
file (GLOB CPP_FILES 
  src/bindings.cpp
  src/drive_controller.cpp

  #
  # Linking SimpleMotion Library source code
  ${SimpleMotionV2_SOURCE_DIR}/sm_consts.c
  ${SimpleMotionV2_SOURCE_DIR}/simplemotion.c
  ${SimpleMotionV2_SOURCE_DIR}/busdevice.c
  ${SimpleMotionV2_SOURCE_DIR}/bufferedmotion.c
  ${SimpleMotionV2_SOURCE_DIR}/devicedeployment.c

  ${SimpleMotionV2_SOURCE_DIR}/utils/crc.c

  ${SimpleMotionV2_SOURCE_DIR}/drivers/serial/pcserialport.c
  ${SimpleMotionV2_SOURCE_DIR}/drivers/tcpip/tcpclient.c
)

# Enable built in drivers
add_definitions(-DENABLE_BUILT_IN_DRIVERS)

############# Compile #############
pybind11_add_module(smv2_drive ${CPP_FILES} ${H_FILES})

target_link_libraries(smv2_drive
  PRIVATE
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
