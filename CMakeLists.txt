find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories( "/usr/include/eigen3" )

find_package( OpenCV REQUIRED )
find_package(realsense2 REQUIRED) 

add_executable(joinMap src/joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${realsense2_LIBRARY} fmt)


