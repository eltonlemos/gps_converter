import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped,Pose2D
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray,Float64
import gpsconv.gps_utils as gps_utils
import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gps_converter')

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('pose_topic', '/pose'),
        #         ('gps_sub_topic', '/gps'),
        #         ('reference_topic', '/reference'),
        #         ('reference_llh', [-1, -1, -1]),
        #         ('pose_frame', 'map')
        #     ]
        # )

        self.declare_parameter('pose_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('gps_sub_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('reference_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('reference_llh',rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('pose_frame',rclpy.Parameter.Type.STRING)
        self.declare_parameter('heading_topic',rclpy.Parameter.Type.STRING)
        



        reference_llh = self.get_parameter("reference_llh").value
        if reference_llh[0] == -1:
            # print("No reference coordinates provided... Waiting for first GPS message to set reference coordinates.")
            self.get_logger().info("No reference coordinates provided... Waiting for first GPS message to set reference coordinates.")
            self.reference_coords = None
        else:
            self.reference_coords = reference_llh

        pose_topic = self.get_parameter("pose_topic").value
        gps_sub_topic = self.get_parameter("gps_sub_topic").value
        reference_topic = self.get_parameter("reference_topic").value
        self.pose_frame = self.get_parameter("pose_frame").value
        heading_topic = self.get_parameter("heading_topic").value

        log_str = f"""
        Pose topic: {pose_topic}
        GPS sub topic: {gps_sub_topic}
        Reference topic: {reference_topic}
        Pose frame: {self.pose_frame}
        Heading topic: {heading_topic}
        """

        self.get_logger().info(log_str)

        self.publisher_ = self.create_publisher(Pose2D, pose_topic, 10)
        self.reference_coords : list = None
        self.gps_subscriber = self.create_subscription(NavSatFix, gps_sub_topic, self.gps_callback, 10)
        self.reference_subscriber = self.create_subscription(Float32MultiArray, reference_topic , self.reference_callback, 10)
        self.heading_subscriber = self.create_subscription(Float64, heading_topic , self.heading_callback, 10)

        self.heading = None

    def reference_callback(self, msg):
        if len(msg.data) > 3:
            raise ValueError("Reference coordinates must be a 3 element array")
        self.reference_coords = msg.data
        print(f"Reference coordinates set to: {self.reference_coords}")

    def heading_callback(self, msg):
        self.get_logger().info(f"Heading: {msg.data}")
        bearing = msg.data

        if bearing > 0 and bearing < 90:
            self.heading = 90 - bearing
        elif bearing > 90 and bearing < 180:
            self.heading = 90 - bearing

        elif bearing > 180 and bearing < 270:
            self.heading = 90 - bearing
        
        elif bearing > 270 and bearing < 360:
            bearing -= 360
            self.heading = 90 - bearing

        # print(f"Heading: {self.heading}")
        self.get_logger().info(f"Yaw: {self.heading}")

        self.heading = math.radians(self.heading)


    def gps_callback(self, msg):
        if self.reference_coords is None:
            self.reference_coords = [msg.latitude, msg.longitude, msg.altitude]
            return
        if self.heading is None or self.heading < 1e-3:
            return
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        coords = [[lat, lon, alt]]

        p = gps_utils.wgs84_2ENU(coords, *self.reference_coords)
        x = p[0, 0]
        y = p[0, 1]
        z = p[0, 2]
        # pose = PoseStamped()
        # pose.header = msg.header
        # pose.header.frame_id = self.pose_frame
        # pose.pose.position.x = x
        # pose.pose.position.y = y
        # pose.pose.position.z = z
        # print(f"Lat: {lat}, Lon: {lon}, Alt: {alt} -> X: {x}, Y: {y}, Z: {z}")
        # self.publisher_.publish(pose)

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = self.heading
        pose.header = msg.header
        self.publisher_.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()