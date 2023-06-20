cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  message_generation
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)

add_message_files(
   FILES
   Message.msg

 )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
generate_messages(
 DEPENDENCIES
   std_msgs  #Or other packages containing msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mobile_robot
  CATKIN_DEPENDS rospy std_msgs roscpp
  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




add_executable(att src/att.cpp)
target_link_libraries(att ${catkin_LIBRARIES})
add_dependencies(att mobile_robot_generate_messages_cpp)

add_executable(att2 src/att2.cpp)
target_link_libraries(att2 ${catkin_LIBRARIES})
add_dependencies(att2 mobile_robot_generate_messages_cpp)

add_executable(wallfollow src/wallfollow.cpp)
target_link_libraries(wallfollow ${catkin_LIBRARIES})
add_dependencies(wallfollow mobile_robot_generate_messages_cpp)

add_executable(oaatt src/oaatt.cpp)
target_link_libraries(oaatt ${catkin_LIBRARIES})
add_dependencies(oaatt mobile_robot_generate_messages_cpp)

add_executable(temp src/temp.cpp)
target_link_libraries(temp ${catkin_LIBRARIES})
add_dependencies(temp mobile_robot_generate_messages_cpp)

add_executable(pclonly src/pclonly.cpp)
target_link_libraries(pclonly ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_dependencies(pclonly mobile_robot_generate_messages_cpp)


