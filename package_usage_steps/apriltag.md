# AprilTag 

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0" -p pixel_format:="mjpeg2rgb" -p camera_info_url:="file://$HOME/calibration_data/ost.yaml" -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info 

ros2 run apriltag_ros apriltag_node --ros-args -r /camera/image_rect:='/camera/image_raw' -r /camera/camera_info:='/camera/camera_info_throttled' --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml

ros2 run topic_tools throttle messages /camera/camera_info 5.0 /camera/camera_info_throttled


[image_transport] Topics '/image_rect' and '/camera_info' do not appear to be synchronized. In the last 10s:

```
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
import image_transport

from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo


class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')
        qos = QoSProfile(depth=10)
       
       	it = image_transport.ImageTransport(rospy)
        self.image_sub = it.Subscriber(self, Image, "image_rect")
        self.info_sub = Subscriber(self, CameraInfo, "camera_info")
        # self.synced_image_pub = self.create_publisher(Image, 'image_rect', qos)
        # self.synced_info_pub = self.create_publisher(CameraInfo, 'camera_info', qos)
	
        queue_size = 10000
        # max_delay = 0.5
        self.time_sync = TimeSynchronizer([self.image_sub, self.info_sub],
                                                     queue_size)
        self.time_sync.registerCallback(self.SyncCallback)
   
    def SyncCallback(self, image, camera_info):
    	image_sec = image.header.stamp.sec
    	info_sec = camera_info.header.stamp.sec
    	self.get_logger().info(f'Sync callback with {image_sec} and {info_sec} as times')
    	#self.synced_image_pub.publish(image)
    	#self.synced_info_pub.publish(camera_info)
    	

def main(args=None):
    rclpy.init(args=args)

    time_sync = TimeSyncNode()
    try:	
    	rclpy.spin(time_sync)
    except KeyboardInterrupt:
    	pass
    finally:
    	time_sync.destroy_node()
    	rclpy.shutdown()


if __name__ == '__main__':
    main()
   
```
