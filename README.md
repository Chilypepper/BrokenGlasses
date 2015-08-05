# brokenGlasses
Vision that works, but isn't perfect. 

A first attempt at vision processing using ROS and the OpenCV library.

[Friendly script to set up everything](https://github.com/Chilypepper/ros-opencv-setup)

Instructions:

1. Run pg.launch as a super user inside the camera_launches folder

2. $ ROS_NAMESPACE=camera rosrun image_proc image_proc
   ***Must be done with a pre-calibrated camera*** 
3. $ rosrun image_testing cv_modder

4. $ rosrun image_view image_view image:=/image_converter/output_video

/ 5. Subscribe your node to the topic "/camera/image_rect_color" for a rectified, colored image in a good form usable with OpenCV
