cmake_minimum_required(VERSION 3.10)

project(pcl_tutorial)

set(CMAKE_CXX_STANDARD 17)



find_package(Boost 1.58 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        src
        # ${catkin_INCLUDE_DIRS}
)

file(COPY ./materials/
        DESTINATION ./auxiliary/
        FILES_MATCHING
        PATTERN *.bin)



add_executable(lec07_sor lec07_sor.cpp)
target_link_libraries(lec07_sor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_package() 
# ToDo
# ICP, G-ICP, RANSAC







