cmake_minimum_required(VERSION 2.8.3)
project(fremen)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg octomap_server)
set ( CMAKE_CXX_FLAGS "-O3 -march=native")

include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 

add_service_files(
		FILES
		GenerateOctomap.srv
		SaveGrid.srv
		UpdateGrid.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs genmsg)

add_executable(save_client src/save_client.cpp)
add_dependencies(save_client genmsg fremen_generate_messages)
target_link_libraries(save_client ${catkin_LIBRARIES})

add_executable(update_client src/update_client.cpp)
add_dependencies(update_client  genmsg fremen_generate_messages)
target_link_libraries(update_client ${catkin_LIBRARIES})

add_executable(octomap_client src/octomap_client.cpp)
target_link_libraries(octomap_client ${catkin_LIBRARIES})
add_dependencies(octomap_client  genmsg fremen_generate_messages)

add_executable(froctomap src/froctomap.cpp)
add_executable(octomap_markers src/octomap_markers.cpp)
add_executable(publish_data src/publish_data.cpp)

add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fftplan src/CFFTPlan.cpp)
add_library(fremengrid src/CFremenGrid.cpp)

target_link_libraries(frelement fftw3)
target_link_libraries(frelement fftplan)
target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengrid frelement)
target_link_libraries(froctomap fremengrid)
target_link_libraries(publish_data ${catkin_LIBRARIES})


target_link_libraries(froctomap ${catkin_LIBRARIES})
target_link_libraries(octomap_markers ${catkin_LIBRARIES})


