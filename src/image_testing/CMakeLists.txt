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
add_executable(cv_modder src/cv_modder.cpp)

target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(cv_modder ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})