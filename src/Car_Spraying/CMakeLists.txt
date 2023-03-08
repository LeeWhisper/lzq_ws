cmake_minimum_required(VERSION 3.0.2)
project(Car_Spraying)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /usr/local/OpenCV/share/OpenCV)
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  std_msgs
  roscpp
  rospy
  image_transport
  cv_bridge
  PCL
  message_generation # Message生成依赖
  tf
  message_filters

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   ros_msgs
# )

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




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



# 设置串口总线舵机链接库的位置
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
)

add_message_files(
  FILES
  SetTwoAngle.msg
  SetServoAngle.msg     # 设置舵机角度
  SetServoDamping.msg   # 设置舵机为阻尼模式
)

add_service_files(
  FILES
  QueryServoAngle.srv  # 舵机角度查询服务
  
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES Car_Spraying
 CATKIN_DEPENDS image_transport roscpp rospy cv_bridge message_runtime std_msgs message_filters
#  DEPENDS system_lib
)

set(CMAKE_INSTALL_PREFIX /usr/local)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Car_Spraying.cpp
# )

add_library(MeanFiter
  include/Car_Spraying/MeanFiter.h
  src/MeanFiter.cpp
)
add_dependencies(MeanFiter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MeanFiter
  ${catkin_LIBRARIES}
)

add_library(fitting
  include/Car_Spraying/fitting.h
  src/fitting.cpp
)
add_dependencies(fitting ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fitting
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(image src/image.cpp)
target_link_libraries(image
  ${catkin_LIBRARIES}
)

add_executable(Sig_id src/Sig_id.cpp)
target_link_libraries(Sig_id
  ${catkin_LIBRARIES}
)

add_executable(color_sepration src/color_sepration.cpp)
target_link_libraries(color_sepration
  ${catkin_LIBRARIES}
)

add_executable(output_cloud src/output_cloud.cpp)
target_link_libraries(output_cloud
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_comparate src/pointcloud_comparate.cpp)
target_link_libraries(pointcloud_comparate
  ${catkin_LIBRARIES}
)

add_executable(pairwise_incremental_registration src/pairwise_incremental_registration.cpp)
target_link_libraries(pairwise_incremental_registration
  ${catkin_LIBRARIES}
)

add_executable(cloud_transform src/cloud_transform.cpp)
target_link_libraries(cloud_transform
  ${catkin_LIBRARIES}
)

add_executable(img_test src/img_test.cpp)
target_link_libraries(img_test
  ${catkin_LIBRARIES}
)

add_executable(uservo_move src/uservo_move.cpp)
target_link_libraries(uservo_move
  ${catkin_LIBRARIES}
  cserialport # 轻量级串口通信库
  fsuartservo # FashionStar串口总线舵机SDK (C++)
)

add_executable(uservo_demo_node src/uservo_demo_node.cpp)
target_link_libraries(uservo_demo_node
  ${catkin_LIBRARIES}
  cserialport # 轻量级串口通信库
  fsuartservo # FashionStar串口总线舵机SDK (C++)
)

add_executable(uservo_move_feedback src/uservo_move_feedback.cpp)
target_link_libraries(uservo_move_feedback
  ${catkin_LIBRARIES}
  cserialport # 轻量级串口通信库
  fsuartservo # FashionStar串口总线舵机SDK (C++)
)

add_executable(read_rgb src/read_rgb.cpp)
target_link_libraries(read_rgb
  ${catkin_LIBRARIES}
)

add_executable(image_segmentation src/image_segmentation.cpp)
target_link_libraries(image_segmentation
  ${catkin_LIBRARIES}
)

add_executable(imagesub src/imagesub.cpp)
target_link_libraries(imagesub
  ${catkin_LIBRARIES}
)

add_executable(calculate src/calculate.cpp)
target_link_libraries(calculate
  ${catkin_LIBRARIES}
)

add_executable(actionGreedPT src/actionGreedPT.cpp)
target_link_libraries(actionGreedPT
  ${catkin_LIBRARIES}
)

add_executable(statisticalOutlierRemoval src/statisticalOutlierRemoval.cpp)
target_link_libraries(statisticalOutlierRemoval
  ${catkin_LIBRARIES}
)

add_executable(radiusOutlierRemoval src/radiusOutlierRemoval.cpp)
target_link_libraries(radiusOutlierRemoval
  ${catkin_LIBRARIES}
)

add_executable(cloud_height src/cloud_height.cpp)
target_link_libraries(cloud_height
  ${catkin_LIBRARIES}
)

add_executable(bilateralFilter src/bilateralFilter.cpp)
target_link_libraries(bilateralFilter
  ${catkin_LIBRARIES}
)

add_executable(voxelGrid src/voxelGrid.cpp)
target_link_libraries(voxelGrid
  ${catkin_LIBRARIES}
)

add_executable(deletRepeatPoint_v1 src/deletRepeatPoint_v1.cpp)
target_link_libraries(deletRepeatPoint_v1
  ${catkin_LIBRARIES}
)

add_executable(deletOutline src/deletOutline.cpp)
target_link_libraries(deletOutline
  ${catkin_LIBRARIES}
)

add_executable(MF src/MF.cpp)
target_link_libraries(MF
  MeanFiter
  ${catkin_LIBRARIES}
)

add_executable(point_line src/point_line.cpp)
target_link_libraries(point_line
  ${catkin_LIBRARIES}
)

add_executable(output_txtfile src/output_txtfile.cpp)
target_link_libraries(output_txtfile
  ${catkin_LIBRARIES}
)

add_executable(vertex_bind src/vertex_bind.cpp)
target_link_libraries(vertex_bind
  MeanFiter
  ${catkin_LIBRARIES}
)

add_executable(cloud_sort src/cloud_sort.cpp)
target_link_libraries(cloud_sort
  ${catkin_LIBRARIES}
)

add_executable(LatitudeLines src/LatitudeLines.cpp)
target_link_libraries(LatitudeLines
MeanFiter
  ${catkin_LIBRARIES}
)

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac
  ${catkin_LIBRARIES}
)

add_executable(alpha_shape src/alpha_shape.cpp)
target_link_libraries(alpha_shape
  ${catkin_LIBRARIES}
)

add_executable(color_restrain src/color_restrain.cpp)
target_link_libraries(color_restrain
  ${catkin_LIBRARIES}
)

add_executable(hole_fix src/hole_fix.cpp)
target_link_libraries(hole_fix
  ${catkin_LIBRARIES}
)

add_executable(pcd2ply src/pcd2ply.cpp)
target_link_libraries(pcd2ply
  ${catkin_LIBRARIES}
)

add_executable(pcd2obj src/pcd2obj.cpp)
target_link_libraries(pcd2obj
  ${catkin_LIBRARIES}
)

add_executable(pcd2off src/pcd2off.cpp)
target_link_libraries(pcd2off
  ${catkin_LIBRARIES}
)

add_executable(ApproximateVG src/ApproximateVG.cpp)
target_link_libraries(ApproximateVG
  ${catkin_LIBRARIES}
)

add_executable(UniformSampling src/UniformSampling.cpp)
target_link_libraries(UniformSampling
  ${catkin_LIBRARIES}
)

add_executable(FL src/FL.cpp)
target_link_libraries(FL
fitting
  ${catkin_LIBRARIES}
)
# add_executable(MF src/MF.cpp)
# target_link_libraries(MF
#   ${catkin_LIBRARIES}
# )
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Car_Spraying.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
