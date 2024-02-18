# Install script for directory: /home/gy/DAGSfM_merge_v3/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/colmap" TYPE STATIC_LIBRARY FILES "/home/gy/DAGSfM_merge_v3/src/libcolmap.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/colmap" TYPE STATIC_LIBRARY FILES "/home/gy/DAGSfM_merge_v3/src/libcolmap_cuda.a")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/gy/DAGSfM_merge_v3/src/base/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/clustering/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/controllers/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/estimators/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/exe/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/feature/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/graph/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/map_reduce/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/math/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/optim/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/ransac/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/retrieval/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/rotation_estimation/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/sfm/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/solver/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/ui/cmake_install.cmake")
  include("/home/gy/DAGSfM_merge_v3/src/util/cmake_install.cmake")

endif()

