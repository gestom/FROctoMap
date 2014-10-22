cmake_minimum_required(VERSION 2.8.3)
project(fremen)


find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg octomap_server sensor_msgs)
set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb ")
#set ( CMAKE_CXX_FLAGS "-ggdb")

include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 

add_action_files(FILES froctomap.action information.action)

add_service_files(
		FILES
		RecoverOctomap.srv
		EstimateOctomap.srv
		SaveGrid.srv
		UpdateGrid.srv
		EvaluateGrid.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs actionlib_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime pcl_conversions pcl_ros std_msgs genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 


#add_executable(froctomap_ActionServer src/froctomap_ActionServer.cpp)
#target_link_libraries(froctomap_ActionServer ${catkin_LIBRARIES})
#add_dependencies(froctomap_ActionServer ${fremen_EXPORTED_TARGETS})


#add_executable(froctomap_ActionClient src/froctomap_ActionClient.cpp)
#target_link_libraries(froctomap_ActionClient ${catkin_LIBRARIES})
#add_dependencies(froctomap_ActionClient ${fremen_EXPORTED_TARGETS})


add_executable(froctomap src/froctomap.cpp)
add_executable(information_gain src/information_gain.cpp)
#add_executable(convert_raw src/convert_raw.cpp)
add_executable(froctomap_util src/froctomap_util.cpp)

add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fftplan src/CFFTPlan.cpp)
add_library(fremengrid src/CFremenGrid.cpp)
target_link_libraries(information_gain fremengrid)

target_link_libraries(frelement fftw3)
target_link_libraries(frelement fftplan)
target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengrid frelement)
target_link_libraries(froctomap fremengrid)
target_link_libraries(froctomap_util fremengrid)
target_link_libraries(information_gain ${catkin_LIBRARIES})
#target_link_libraries(convert_raw ${catkin_LIBRARIES})
target_link_libraries(froctomap ${catkin_LIBRARIES})
