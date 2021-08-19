cmake_minimum_required(VERSION 3.0.2)
project(cloud_features)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nodelet
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cloud 
  CATKIN_DEPENDS roscpp sensor_msgs nodelet
#  DEPENDS system_lib
)

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/cloud_features_node.cpp)
set(NODELET_SRC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/nodelet/cloud_features_nodelet.cpp
)

###############################################################################

###############################################################################
# INCLUDES

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###############################################################################

###############################################################################
# EXECUTABLE

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
)

add_library(CloudFeaturesNodelet ${NODELET_SRC})
target_link_libraries(CloudFeaturesNodelet ${LINK_LIBRARIES})
add_dependencies(CloudFeaturesNodelet ${catkin_EXPORTED_TARGETS})


add_executable(cloud_features_node ${NODE_SRC})
target_link_libraries(cloud_features_node CloudFeaturesNodelet ${LINK_LIBRARIES})

###############################################################################

###############################################################################
# INSTALL

install(TARGETS
  CloudFeaturesNodelet
  cloud_features_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

