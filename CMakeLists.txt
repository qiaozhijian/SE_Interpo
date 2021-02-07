#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.5)
project(SE_Interpo)
message("start make cmake_template")

# 一定要加-march=native， 不然使用pcl的一些函数会报错
#add_definitions(-DCPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -march=native")

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_defination.cmake)
include(cmake/OpenCV.cmake)
include(cmake/eigen.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
)


file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
#file(GLOB_RECURSE THIRD_PARTY_SRCS "Thirdparty/*.cpp")
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


# lidar slam and localize
add_executable(main exec/main.cpp ${ALL_SRCS})
target_link_libraries(main ${ALL_TARGET_LIBRARIES})

