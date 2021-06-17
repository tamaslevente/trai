cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ppfplane)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (openni_grabber openni_grabber/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (create_planes Final/create_planes.cpp)
target_link_libraries (create_planes ${PCL_LIBRARIES})

add_executable (distante_final Final/distante_final.cpp)
target_link_libraries (distante_final ${PCL_LIBRARIES})

add_executable (linii_final Final/linii_final.cpp)
target_link_libraries (linii_final ${PCL_LIBRARIES})

add_executable (proiectii_final Final/proiectii_final.cpp)
target_link_libraries (proiectii_final ${PCL_LIBRARIES})
<<<<<<< HEAD

add_executable (find_line Intersectii/find_line.cpp)
target_link_libraries (find_line ${PCL_LIBRARIES})

add_executable (find_line_all Intersectii/find_line_all.cpp)
target_link_libraries (find_line_all ${PCL_LIBRARIES})

add_executable (find_intersections_line Intersectii/find_intersections_line.cpp)
target_link_libraries (find_intersections_line ${PCL_LIBRARIES})
=======
>>>>>>> 24f7e2be110b09f6da7b130ab05ee154d3b55ea9
