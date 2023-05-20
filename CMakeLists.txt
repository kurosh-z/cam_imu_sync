cmake_minimum_required(VERSION 3.0.2)
project(imu_cam_sync)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11")
add_compile_options(-std=c++11)



## Find catkin macros and libraries
find_package(Boost REQUIRED COMPONENTS thread system)
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
  imu_cam_msgs
)

catkin_destinations()




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)

set(IMU_CAM_SYNC_NODELET_NAME ic_sync_nodelet)
set(IMU_CAM_SYNC_LIB_NAME isync_serial)

set(IMU_CAM_SYNC_NODELET_SOURCES
src/ic_sync_nodelet.cpp
)

set(IMU_CAM_SYNC_NODELET_HEADERS
include/imu_cam_sync/ic_sync_nodelet.hpp
)

set (IMU_CAM_SYNC_LIB_SOURCES 
src/ic_sync_serial.cpp

)
set (IMU_CAM_SYNC_LIB_HEADERS 
include/imu_cam_sync/ic_sync_serial.hpp
include/imu_cam_sync/thread_utils.hpp
include/imu_cam_sync/bytes_utils.hpp
include/imu_cam_sync/TimestampBuff.hpp
)

# add_message_files(DIRECTORY msg
#   FILES
#   ICtrigger.msg
# )

# add_service_files(DIRECTORY srv
# FILES
# cmd_imu.srv
# )

# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} ${IMU_CAM_SYNC_NODELET_NAME} ${IMU_CAM_SYNC_LIB_NAME} 
 CATKIN_DEPENDS roscpp nodelet  std_srvs std_msgs message_runtime imu_cam_msgs
 DEPENDS Boost
)


##build

####################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

  )


# add_library(${IMU_CAM_SYNC_NODELET_NAME} ${IMU_CAM_SYNC_NODELET_SOURCES} ${IMU_CAM_SYNC_NODELET_HEADERS})
add_library(${IMU_CAM_SYNC_LIB_NAME} src/ic_sync_serial.cpp include/imu_cam_sync/ic_sync_serial.hpp include/imu_cam_sync/thread_utils.hpp include/imu_cam_sync/bytes_utils.hpp include/imu_cam_sync/TimestampBuff.hpp)
SET_TARGET_PROPERTIES(${IMU_CAM_SYNC_LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${IMU_CAM_SYNC_LIB_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}  src/ic_sync_nodelet.cpp  include/imu_cam_sync/ic_sync_nodelet.hpp )


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})



target_link_libraries(${PROJECT_NAME} ${IMU_CAM_SYNC_LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES} )



add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

##install


install(TARGETS ${PROJECT_NAME} ${IMU_CAM_SYNC_LIB_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )