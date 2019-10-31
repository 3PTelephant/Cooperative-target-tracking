cmake_minimum_required(VERSION 2.8.3)
project(cotrack)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    dynamic_reconfigure
   ##如果要把自己这个包加进去一定要保证package.xml也要改
)
find_package(OpenCV)
find_package(Eigen3)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   drone_state.msg
#   Message2.msg
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cotrack
 CATKIN_DEPENDS message_runtime  std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#------------------------DETECTION_PART-------------------------#
set(TARGETDETECTION_SOURCE_FILES
      src/TargetDetection/main_targetdetection.cpp
      src/TargetDetection/TargetDetection.cpp
    )
set(TARGETDETECTION_HEADER_FILES
     src/TargetDetection/TargetDetection.h
    )
add_executable(target_detect ${TARGETDETECTION_SOURCE_FILES} ${TARGETDETECTION_HEADER_FILES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(target_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(target_detect
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

#------------------------STATE ESTIMATION-------------------------#
set(STATEESTIMATION_SOURCE_FILES
      src/StateEstimation/main_stateestimation.cpp
      src/StateEstimation/StateEstimation.cpp
    )
set(STATEESTIMATION_HEADER_FILES
     src/StateEstimation/StateEstimation.h
    )
add_executable(state_estimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(state_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(state_estimation
   ${catkin_LIBRARIES}
 )


#------------------------KALMAN FILTER-------------------------#
set(KALMANFILTER_SOURCE_FILES
      src/KalmanFilter/main_kalmanfilter.cpp
      src/KalmanFilter/KalmanFilter.cpp
    )
set(KALMANFILTER_HEADER_FILES
     src/KalmanFilter/KalmanFilter.h
    )
add_executable(kalman_filter ${KALMANFILTER_SOURCE_FILES} ${KALMANFILTER_HEADER_FILES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(kalman_filter
   ${catkin_LIBRARIES}
 )




#--------------------------Control Demo Part-----------------------#

add_executable(bebop_track4 src/Cotrack/bebop_track4.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(bebop_track4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(bebop_track4
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )




add_executable(bebop_track3 src/Cotrack/bebop_track3.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(bebop_track3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(bebop_track3
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

add_executable(bebop_track2 src/Cotrack/bebop_track2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(bebop_track2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(bebop_track2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

add_executable(bebop_track1 src/Cotrack/bebop_track1.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(bebop_track1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(bebop_track1
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
add_executable(bebop_singtrack src/Cotrack/bebop_singtrack.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(bebop_singtrack ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(bebop_singtrack
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )



add_executable(statemachine src/Cotrack/statemachine_node.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(statemachine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(statemachine
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )




add_executable(pnpcompare src/PnPcompare/pnpcompare_node.cpp src/PnPcompare/RPnP.h src/PnPcompare/RPnP.cpp  )
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pnpcompare ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pnpcompare
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


add_executable(pnpcompare_RPnp_homo src/PnPcompare/pnpcompare.cpp src/PnPcompare/RPnP.h  src/PnPcompare/RPnP.cpp src/PnPcompare/RPP.h src/PnPcompare/Rpoly.h src/PnPcompare/Rpoly.cpp  src/PnPcompare/RPP.cpp  )
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pnpcompare_RPnp_homo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pnpcompare_RPnp_homo
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

add_executable(pnpcompare_RPP src/PnPcompare/demo.cpp src/PnPcompare/RPP.h src/PnPcompare/Rpoly.h src/PnPcompare/Rpoly.cpp  src/PnPcompare/RPP.cpp  )
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pnpcompare_RPP ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pnpcompare_RPP

   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )



