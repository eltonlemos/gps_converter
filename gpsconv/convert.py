import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import gpsconv.gps_utils as gps_utils
import math
from swiftnav_ros2_driver.msg import Baseline
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np

def quaternion_to_euler(qx,qy,qz,qw):
    r = R.from_quat([qx,qy,qz,qw])
    return r.as_euler('xyz', degrees=False)

def euler_to_quaternion(roll,pitch,yaw):
    r = R.from_euler('xyz', [roll,pitch,yaw], degrees=False)
    return r.as_quat()

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
        self.tf_broadcaster = TransformBroadcaster(self)

        self.declare_parameter('pose_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('gps_sub_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('reference_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('reference_llh',rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('pose_frame',rclpy.Parameter.Type.STRING)
        self.declare_parameter('heading_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('twist_topic',rclpy.Parameter.Type.STRING)
        self.declare_parameter('constant_heading_offset',0.0)
        



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
        self.constant_heading_offset = self.get_parameter("constant_heading_offset").value
        twist_topic = self.get_parameter("twist_topic").value

        log_str = f"""
        Pose topic: {pose_topic}
        GPS sub topic: {gps_sub_topic}
        Reference topic: {reference_topic}
        Pose frame: {self.pose_frame}
        Heading topic: {heading_topic}
        Constant heading offset: {self.constant_heading_offset}
        Twist topic: {twist_topic}
        """

        self.get_logger().info(log_str)

        self.publisher_ = self.create_publisher(Odometry, pose_topic, 10)
        self.reference_coords : list = None
        self.gps_subscriber = self.create_subscription(NavSatFix, gps_sub_topic, self.gps_callback, 10)
        self.heading_subscriber = self.create_subscription(Baseline, heading_topic , self.heading_callback, 10)

        self.heading = None
        self.heading_cov = None
        self.initial_heading = None

        self.frame_origin = None
        self.yaw_offset = None

        self.twist = None

    def genTransform(self,x,y,z,quat):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


    def heading_callback(self, msg:Baseline):
        

        bearing = msg.baseline_dir_deg - self.constant_heading_offset
        if bearing < 0:
            bearing += 360
        if bearing == 0:
            bearing += 1e-2
        # bearing += 360

        self.get_logger().info(f"Dir: {msg.baseline_dir_deg}, Bearing: {bearing}")

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
        self.get_logger().info(f"Heading: {self.heading}")
        self.heading = math.radians(self.heading)

    def rotate_ref(self,p,roll,pitch,yaw):
        
        R = np.array([
            [math.cos(yaw)*math.cos(pitch), math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll), math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)],
            [math.sin(yaw)*math.cos(pitch), math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll), math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll)],
            [-math.sin(pitch), math.cos(pitch)*math.sin(roll), math.cos(pitch)*math.cos(roll)]
        ])

        p = np.array(p)
        p = np.dot(R,p)
        return p
    
    # def coordinate_transform(self,frame_origin_xyt,point_xyt):
    #     x = point_xyt[0] - frame_origin_xyt[0]
    #     y = point_xyt[1] - frame_origin_xyt[1]
        
    #     yaw_temp = frame_origin_xyt[2]
        
    #     x_local = x*np.cos(-yaw_temp) - y*np.sin(-yaw_temp)
    #     y_local = x*np.sin(-yaw_temp) + y*np.cos(-yaw_temp)

    #     yaw_local = point_xyt[2] - frame_origin_xyt[2]
    #     return [x_local,y_local,yaw_local]

    def coordinate_transform(self, global_local_origin, global_local_point):
        x = global_local_point[0] - global_local_origin[0]
        y = global_local_point[1] - global_local_origin[1]
        theta = global_local_origin[2]
        x_local = x*np.cos(theta) + y*np.sin(theta)
        y_local = -x*np.sin(theta) + y*np.cos(theta)
        yaw_local = global_local_point[2] - global_local_origin[2]
        return [x_local, y_local, yaw_local]


    def gps_callback(self, msg:NavSatFix):
        if self.reference_coords is None:
            self.reference_coords = [msg.latitude, msg.longitude, msg.altitude]

        if self.heading is None:
            return
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        coords = [[lat, lon, alt]]

        p = gps_utils.wgs84_2ENU(coords, *self.reference_coords)
        x = p[0, 0]
        y = p[0, 1]
        z = p[0, 2]

        if self.frame_origin is None:
            self.frame_origin = [x,y,self.heading]
            self.get_logger().info(f"Initalizing frame origin to: {self.frame_origin}")

        position_covariance = [0.0] * 36
        
        for i in range(9):
            position_covariance[i] = msg.position_covariance[i]

        for i in range(9,36):
            position_covariance[i] = 0.5

        p = [x,y,self.heading]

        x,y,yaw = self.coordinate_transform(self.frame_origin,p)
        self.get_logger().info(f"X: {x}, Y: {y}, Yaw: {yaw}")
        quat = euler_to_quaternion(0,0,yaw)


        # pose_cov = PoseWithCovarianceStamped()
        # pose_cov.header = msg.header
        # pose_cov.header.frame_id = self.pose_frame
        # pose_cov.pose.pose.position.x = x
        # pose_cov.pose.pose.position.y = y
        # pose_cov.pose.pose.position.z = z
        # pose_cov.pose.pose.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        # pose_cov.pose.covariance = position_covariance

        # self.publisher_.publish(pose_cov)

        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = self.pose_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        odom_msg.pose.covariance = position_covariance

        if self.twist is not None:
            odom_msg.twist.twist = self.twist

        self.publisher_.publish(odom_msg)

        self.genTransform(x,y,z,quat)


        # print(f"X: {x}, Y: {y}, Z: {z}")
        # self.get_logger().info(f"X: {x}, Y: {y}, Z: {z}\n\n")


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