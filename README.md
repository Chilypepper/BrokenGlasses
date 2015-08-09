# brokenGlasses
Vision that works, but isn't perfect. 

A first attempt at vision processing using ROS and the OpenCV library.

[Friendly script to set up everything](https://github.com/Chilypepper/ros-opencv-setup)

Instructions:

1. Run pg.launch as a super user inside the camera_launches folder

  -- Done!

** Both cameras not working at the same time?

$ subl /etc/defalt/grub

change GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"

to

GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"

**Answer to the superuser issue

http://askubuntu.com/questions/41402/how-do-i-get-opencv-and-firefly-mv-working
