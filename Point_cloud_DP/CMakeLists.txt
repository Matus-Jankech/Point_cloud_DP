cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(Point_cloud_DP)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
 )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(main ${all_SRCS} "src/cloud_handler.cpp" "include/cloud_handler.h")
target_link_libraries(main ${PCL_LIBRARIES})

# Set C++ standard
target_compile_features(main PUBLIC cxx_std_17)