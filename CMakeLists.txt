cmake_minimum_required(VERSION 3.10)

project(sor)

set(CMAKE_CXX_STANDARD 17)


find_package(Boost 1.58 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
        src
        # ${catkin_INCLUDE_DIRS}
)





add_executable(lec07_sor lec07_sor.cpp)
target_link_libraries(lec07_sor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


# ToDo
# ICP, G-ICP, RANSAC







