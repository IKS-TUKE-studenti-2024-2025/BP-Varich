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

from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import CompressedImage, CameraInfo


class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')
        qos = QoSProfile(depth=10)
       
        self.image_sub = Subscriber(self, CompressedImage, '/image_raw/compressed')
        self.info_sub = Subscriber(self, CameraInfo, "/camera_info_t")
        # self.synced_image_pub = self.create_publisher(CompressedImage, '/image_rect', qos)
        # self.synced_info_pub = self.create_publisher(CameraInfo, '/camera_info', qos)
	
        queue_size = 300
        # max_delay = 0.5
        self.time_sync = TimeSynchronizer([self.image_sub, self.info_sub],
                                                     queue_size)
        self.time_sync.registerCallback(self.SyncCallback)
   
    def SyncCallback(self, image, camera_info):
    	image_sec = image.header.stamp.sec
    	info_sec = camera_info.header.stamp.sec
    	self.get_logger().info(f'Sync callback with {image_sec} and {info_sec} as times')
    	# self.synced_image_pub.publish(image)
    	# self.synced_info_pub.publish(camera_info)
    	

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

```
AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam(image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_raw/compressed"), // ! image_rect
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "compressed", descr({}, true)), // raw
        rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
```

`ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0" -p pixel_format:="mjpeg2rgb" -p camera_info_url:="file://$HOME/calibration_data/ost.yaml" -r camera_info:=/camera_info_t -p image_transport:='compressed' -r image_raw/compressed:=/image_raw/compressed`

`ros2 run apriltag_ros apriltag_node --ros-args -r image_raw/compressed:=/image_raw/compressed -r camera_info:=/camera_info_t  --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml `

`ros2 run topic_tools throttle messages /camera_info_throttled 2.0 /camera_info_t`

`ros2 run time_sync_node sync_node`

```
// Установим параметр image_transport перед подпиской
    std::string transport = this->declare_parameter("image_transport", "compressed");

    // Инициализация подписки на камеру с учетом параметра transport
    sub_cam = image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_raw"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        transport,
        rmw_qos_profile_sensor_data
    );
```

