cmake_minimum_required(VERSION 2.8)
project(All_toghether)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(add_normals add_normals.cpp)
target_link_libraries(add_normals ${PCL_LIBRARIES})

add_executable(pcd_density pcd_density.cpp)
target_link_libraries(pcd_density ${PCL_LIBRARIES})

add_executable(pcd_downsample pcd_downsample.cpp)
target_link_libraries(pcd_downsample ${PCL_LIBRARIES})

add_executable(pcd_read pcd_read.cpp)
target_link_libraries(pcd_read ${PCL_LIBRARIES})

add_executable(batch_add_normals batch_add_normals.cpp)
target_link_libraries(batch_add_normals ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_eliminate_nan_points batch_eliminate_nan_points.cpp)
target_link_libraries(batch_eliminate_nan_points ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_add_normals_voxel batch_add_normals_voxel.cpp)
target_link_libraries(batch_add_normals_voxel ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel batch_voxel.cpp)
target_link_libraries(batch_voxel ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_nan_remove batch_nan_remove.cpp)
target_link_libraries(batch_nan_remove ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel_passthrough batch_voxel_passthrough.cpp)
target_link_libraries(batch_voxel_passthrough ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel_passthrough_double_iterator batch_voxel_passthrough_double_iterator.cpp)
target_link_libraries(batch_voxel_passthrough_double_iterator ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel_passthrough_score batch_voxel_passthrough_score.cpp)
target_link_libraries(batch_voxel_passthrough_score ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel_passthrough_score_Volume batch_voxel_passthrough_score_Volume.cpp)
target_link_libraries(batch_voxel_passthrough_score_Volume ${PCL_LIBRARIES} stdc++fs)

add_executable(batch_voxel_passthrough_double_iterator_same_name batch_voxel_passthrough_double_iterator_same_name.cpp)
target_link_libraries(batch_voxel_passthrough_double_iterator_same_name ${PCL_LIBRARIES} stdc++fs)

add_executable(Reeb_graph Reeb_graph.cpp)
target_link_libraries(Reeb_graph ${PCL_LIBRARIES} stdc++fs)






