import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

class TopicHzReader(Node):
    def __init__(self):
        super().__init__('topic_hz_reader')
        
        self.prev_time = 0
        self.current_time = 0
        self.topic_frequency = 0
        self.fps = 0
        self.first = True
        
        self.subscription = self.create_subscription(
            Image,  # Replace with the actual message type you're subscribing to
            '/front/stereo_camera/left/rgb',  # Replace with the actual topic name
            self.check_hz,
            10,
            # QoSProfile(depth=10)
        )
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Frequency: {self.topic_frequency:.2f} Hz - FPs: {self.fps:.2f}")

    def check_hz(self, msg):

        if self.first:
            self.prev_time = self.get_clock().now()
            self.first = False
        else:
            self.current_time = self.get_clock().now()
            elapsed_time = self.current_time - self.prev_time
            if elapsed_time.nanoseconds > 0:
                self.topic_frequency = 1e9 / elapsed_time.nanoseconds
            self.prev_time = self.current_time
    
    def viewpoint(self, value):
        self.fps = value
# EOF