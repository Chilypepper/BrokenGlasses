cmake_minimum_required(VERSION 2.8.3)
project(image_testing)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
image_transport
cv_bridge
rosconsole
sensor_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} )

add_executable(image_converter src/image_converter.cpp)
add_executable(green_finder src/cv_modder.cpp)
add_executable(circle_finder src/circle_finder.cpp)
add_executable(rect_finder src/rect_finder.cpp)
add_executable(orange_finder src/orange_finder.cpp)
add_executable(hsv_converter src/hsv_converter.cpp)
add_executable(color_finder_average src/color_finder_average.cpp)


target_link_libraries(color_finder_average ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(hsv_converter ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(rect_finder ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(circle_finder ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(green_finder ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(orange_finder ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})