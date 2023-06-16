import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

from math import sin, cos, pi

from std_msgs.msg import String, Header
from rosgraph_msgs.msg import Clock as Clock_ros
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose, PoseStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, LaserScan, JointState
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, TransformStamped

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


import cv2

data = []

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #self.publisher_ = self.create_publisher(String, 'a', 10)
        qos_profile = QoSProfile(depth=10)
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_lidar = self.create_publisher(LaserScan, 'laser_controller/out', 10)
        self.publisher_camera = self.create_publisher(Image, 'camera/image_raw', 10)
        self.publisher_joint = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher_nav2 = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.sub_robot_model = self.create_subscription(String, 'robot_description', self.callback_robot_model,10)
        self.robot_model = String()

        self.action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        #self.subscriber_time = self.create_subscription(Clock_ros, 'clock', self.callback_time, 10 )
        #self.time = Time()
        #self.broadcaster = TransformBroadcaster(self, qos=qos_profile)


        timer_period = 0.05  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    """ def callback_time(self, msg):
        self.time = msg.clock
        print('--------------------')
        print(self.time)
        print('--------------------') """
    
    def callback_robot_model(self, msg):
        self.robot_model = msg
        print('--------------------')
        print(msg)
        print('--------------------')

    def timer_callback(self, angle):
        #cam = cv2.VideoCapture(0)
        #check, frame = cam.read()
        bridge = CvBridge()
        msg = String()
        msg_vel = Twist()
        msg_img = Image()
        msg_lidar = LaserScan()
        msg_joint = JointState()
        msg_nav2 = PoseStamped()
        time = Time()
        angle = 0.0 

        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ['camera_joint_z', 'camera_joint_y']

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [angle, angle]

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback = self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        """  odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'chassis_link'



        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.transform.translation.x = cos(angle)*2
        odom_trans.transform.translation.y = sin(angle)*2
        odom_trans.transform.translation.z = 0.7
        odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

        print('--------------------')

        self.get_logger().info('Odom Transder: "%s"' % odom_trans)


        print('--------------------') """
    

        #--------JointStates-----------
        #msg_joint.header.stamp = self.get_clock().now().to_msg()
        
        
        #msg_joint.header.stamp = self.get_clock().now()
        #sg_joint.header.stamp = self.time
        
        #msg_nav2.header.frame_id = 'map'
        #msg_joint.name = ['L_shoulder_joint_x', 'L_elbow_joint_x', 'camera_joint_z']
        #msg_joint.position = [2.00, 1.20, 0.30]

        """ msg_joint.name = ['camera_joint_z', 'camera_joint_y', 'L_shoulder_joint_y', 'L_shoulder_joint_x', 
                          'L_elbow_joint_z', 'L_elbow_joint_x', 'L_hand_joint_z', 'L_hand_joint_x', 
                          'L_left_gripper_slide_joint', 'L_right_gripper_slide_joint', ]
        msg_joint.position = [0.0, 0.5, 0.79, 1.95,
                              0.52, 1.79, 0.79, -0.79,
                              0.01, 0.01]
        
        msg_joint.velocity = [0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0] """
        
        """ msg_joint.name = ['camera_joint_z', 'camera_joint_y']
        msg_joint.position = [0.0, 0.5]
        
        msg_joint.velocity = [0.0, 1.0]

        self.publisher_joint.publish(msg_joint)
        self.get_logger().info('Publishing: "%s"' % msg_joint) """



        #--------Navigation------------

        """ msg_nav2.header.stamp = self.get_clock().now().to_msg()
        msg_nav2.header.frame_id = 'map'

        msg_nav2.pose.position = Point(x = 1.668, y = 3.128, z = 0.0)
        msg_nav2.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = -0.686, w = 0.726)

        self.publisher_nav2.publish(msg_nav2) """
        

        #-------Velocidade------------

        
        """ msg_vel.angular = Vector3(x = 0.0, y = 0.0, z = -0.5)
        msg_vel.linear = Vector3(x = 0.0, y = 0.0, z = 0.0)
        self.publisher_vel.publish(msg_vel) """
        #self.get_logger().info('Publishing: "%s"' % msg_vel)


        #--------Image-------------

        
        """ msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.header.frame_id = "world"

        path = '~/sim_ws/src/test2/test2/frame.jpeg' """

        #frame = cv2.imread('~/abc.jpg')

        """print(frame)

        print('Original Dimensions : ',frame.shape)
        
        width = int(frame.shape[1])
        height = int(frame.shape[0])

        cv2.imshow('video', frame) """
        
        #cv2.resize(frame, (width, height))
        """ msg_img.height = height
        msg_img.width = width
        msg_img.encoding = "R8G8B8"
        msg_img.is_bigendian = False
        msg_img.step = 3*height
        img = bridge.cv2_to_imgmsg(frame, encoding = "passthrough")

        msg_img.data = img"""

        """ self.publisher_camera.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.get_logger().info('Publishing: "%s"' % frame)


        #---------LIDAR------------
        msg_lidar.header.stamp = self.get_clock().now().to_msg()
        msg_lidar.header.frame_id = "world"

        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 0.0
        msg_lidar.angle_increment = 0.0
        msg_lidar.time_increment = 0.0
        msg_lidar.scan_time = 0.0
        msg_lidar.range_min = 0.0
        msg_lidar.range_max = 0.0
        msg_lidar.ranges = []
        msg_lidar.intensities = []

        self.publisher_lidar.publish(msg_lidar) """
        #self.get_logger().info('Publishing: "%s"' % msg_lidar)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected:(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        
""" def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw) """

def main(args=None):
    rclpy.init(args=args)
    #msg = String()
    minimal_publisher = MinimalPublisher()
    
    future = minimal_publisher.timer_callback(0.5)
    
    rclpy.spin(minimal_publisher)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
