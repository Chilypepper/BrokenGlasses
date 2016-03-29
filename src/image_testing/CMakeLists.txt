cmake_minimum_required(VERSION 2.8.3)
project(image_testing)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
image_transport
cv_bridge
rosconsole
sensor_msgs
point_message
)

add_library(RiptideVision /home/peter/brokenGlasses/src/image_testing/libs/RiptideVision.cpp /home/peter/brokenGlasses/src/image_testing/libs/RiptideVision.h)

#add_executable(image_converter src/image_converter.cpp)
#add_executable(green_finder src/cv_modder.cpp)
#add_executable(circle_finder src/circle_finder.cpp)
#add_executable(rect_finder src/rect_finder.cpp)
#add_executable(orange_finder src/orange_finder.cpp)
#add_executable(hsv_converter src/hsv_converter.cpp)
#add_executable(color_finder_average src/color_finder_average.cpp)
#add_executable(image_flipper src/image_flipper.cpp)
#add_executable(colorSpaces src/colorspaces.cpp)
#add_executable(draw_gui src/draw_gui.cpp)
#add_executable(quad_image src/quad_image.cpp)
#add_executable(horizon_line src/horizonLine.cpp)
add_executable(heading_predict src/heading_marker.cpp)
add_executable(line_follow src/line_follow.cpp)
add_executable(parampicker src/parampicker.cpp)

target_link_libraries(parampicker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(line_follow RiptideVision ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(heading_predict ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
