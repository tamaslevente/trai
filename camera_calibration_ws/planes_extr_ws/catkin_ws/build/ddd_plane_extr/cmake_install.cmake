# Install script for directory: /home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/src/ddd_plane_extr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ddd_plane_extr" TYPE FILE FILES "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/devel/include/ddd_plane_extr/planes_paramConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/ddd_plane_extr" TYPE FILE FILES "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/devel/lib/python2.7/dist-packages/ddd_plane_extr/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/devel/lib/python2.7/dist-packages/ddd_plane_extr/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/ddd_plane_extr" TYPE DIRECTORY FILES "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/devel/lib/python2.7/dist-packages/ddd_plane_extr/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/build/ddd_plane_extr/catkin_generated/installspace/ddd_plane_extr.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ddd_plane_extr/cmake" TYPE FILE FILES
    "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/build/ddd_plane_extr/catkin_generated/installspace/ddd_plane_extrConfig.cmake"
    "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/build/ddd_plane_extr/catkin_generated/installspace/ddd_plane_extrConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ddd_plane_extr" TYPE FILE FILES "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/catkin_ws/src/ddd_plane_extr/package.xml")
endif()

