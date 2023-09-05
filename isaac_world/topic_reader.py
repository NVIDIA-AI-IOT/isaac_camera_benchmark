import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from rclpy.time import Time


class TopicHzReader(Node):
    def __init__(self):
        super().__init__('topic_hz_reader')

        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.window_size = 10000  # window_size
        self.times = []
        self.fps = 0

        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            Image,  # Replace with the actual message type you're subscribing to
            '/front/stereo_camera/left/rgb',  # Replace with the actual topic name
            self.callback_hz,
            qos_profile=qos_profile,
        )
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):

        if not self.times:
            return
        elif self.last_printed_tn == 0:
            self.last_printed_tn = self.msg_tn
            return
        elif self.msg_tn < self.last_printed_tn + 1e9:
            return

        n = len(self.times)
        mean = sum(self.times) / n

        rate = 1. / mean if mean > 0. else 0

        self.last_printed_tn = self.msg_tn

        rate_print = rate * 1e9

        self.get_logger().info(
            f"Frequency: {rate_print:.3f} Hz - FPs: {self.fps:.2f}")

    def callback_hz(self, msg):

        curr_rostime = self.get_clock().now()

        if curr_rostime.nanoseconds == 0:
            if len(self.times) > 0:
                print('time has reset, resetting counters')
                self.times = []
            return

        curr = curr_rostime.nanoseconds
        msg_t0 = self.msg_t0
        if msg_t0 < 0 or msg_t0 > curr:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.times = []
        else:
            self.times.append(curr - self.msg_tn)
            self.msg_tn = curr

        if len(self.times) > self.window_size:
            self.times.pop(0)

    def viewpoint(self, value):
        self.fps = value
# EOF
