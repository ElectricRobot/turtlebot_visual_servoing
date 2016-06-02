cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_visual_servoing)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package( OpenCV REQUIRED )
# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${image_processing}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

# Add personal library
add_library(color_detection src/image_processing/color_detection.cpp)
target_link_libraries (color_detection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
catkin_package(
	DEPENDS VISP
    LIBRARIES color_detection)


## Add executables
add_executable( read_topic src/read_topic.cpp )
target_link_libraries ( read_topic ${catkin_LIBRARIES} )

add_executable( image_transport_subscriber src/image_transport_subscriber.cpp )
target_link_libraries ( image_transport_subscriber ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( image_transport_publisher src/image_transport_publisher.cpp )
target_link_libraries ( image_transport_publisher ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( turtle_turn src/turtle_turn_ok.cpp )
target_link_libraries ( turtle_turn ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( turtle_line src/turtle_line_ok.cpp )
target_link_libraries ( turtle_line ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( turtle_play src/turtle_play.cpp )
target_link_libraries ( turtle_play ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( working_turtle src/workingTurtle.cpp )
target_link_libraries ( working_turtle ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable( turtle_tracker src/turtle_tracker.cpp )
target_link_libraries ( turtle_tracker ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable( turtle_play src/turtle_catenary.cpp )
target_link_libraries ( turtle_play ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

