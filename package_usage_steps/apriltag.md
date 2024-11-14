# AprilTag 

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0" -p pixel_format:="mjpeg2rgb" -p camera_info_url:="file://$HOME/calibration_data/ost.yaml" -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info 

ros2 run apriltag_ros apriltag_node --ros-args -r /camera/image_rect:='/camera/image_raw' -r /camera/camera_info:='/camera/camera_info_throttled' --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml

ros2 run topic_tools throttle messages /camera/camera_info 5.0 /camera/camera_info_throttled


[image_transport] Topics '/image_rect' and '/camera_info' do not appear to be synchronized. In the last 10s:
