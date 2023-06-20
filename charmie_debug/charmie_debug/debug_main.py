import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
    def send_goal(self, angle):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ['upper_body_joint', 'lower_body_joint', 'camera_joint_y', 'camera_joint_z']

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.1745, 0.0, 0.0, 0.0] #start point

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [angle+0.5, angle, angle, angle] #finish point

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected :(')
            return

        self.get_logger().info('Goal Accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        #rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    minimal_publisher.send_goal(0.0)
    
    rclpy.spin(minimal_publisher)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




""" #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Bool, Int16
# import time

class TRNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Node")
        
        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback , 10)
        self.flag_neck_position_publisher = self.create_publisher(Bool, "flag_neck_pos", 10)
        
        # Low Level Topics
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback ,10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        self.vccs_subscriber = self.create_subscription(Pose2D, "get_vccs", self.get_vccs_callback ,10)
        self.flag_vccs_publisher = self.create_publisher(Bool, "flag_vccs", 10)
        
        self.counter = 1 # starts at 1 to avoid initial 
        self.create_timer(4, self.timer_callback)

        self.flag_get_neck_position = False 
        self.flag_get_start_button = False 
        self.flag_get_vccs = False 

    def get_neck_position_callback(self, pos: Pose2D):
        print("Received Neck Position: pan =", int(pos.x), " tilt = ", int(pos.y))

    def get_start_button_callback(self, state: Bool):
        print("Received Start Button: ", state.data)

    def get_vccs_callback(self, vcc: Pose2D):
        print("Received VCC: ", vcc.x, ", and Emergency: ", bool(vcc.y))

    def timer_callback(self):
        cmd = Pose2D()
        flag_neck = Bool()
        flag_start_button = Bool()
        flag_vccs = Bool()
        mode = Int16()

        if self.counter == 0:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = False
            self.flag_get_start_button = False
            self.flag_get_vccs = False
            mode.data = 1
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 1:
            cmd.x = 270.0
            cmd.y = 180.0 
        if self.counter == 2:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = True
            self.flag_get_start_button = True
            self.flag_get_vccs = True
            mode.data = 2
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 3:
            cmd.x = 90.0
            cmd.y = 180.0 
        if self.counter == 4:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = False
            self.flag_get_start_button = False
            self.flag_get_vccs = False
            mode.data = 3
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 5:
            cmd.x = 180.0
            cmd.y = 120.0 
        if self.counter == 6:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = True
            self.flag_get_start_button = True
            self.flag_get_vccs = True
            mode.data = 4
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 7:
            cmd.x = 180.0
            cmd.y = 235.0 
            self.counter = -1

        self.neck_position_publisher.publish(cmd)

        flag_neck.data = self.flag_get_neck_position
        self.flag_neck_position_publisher.publish(flag_neck)

        flag_start_button.data = self.flag_get_start_button
        self.flag_start_button_publisher.publish(flag_start_button)

        flag_vccs.data = self.flag_get_vccs
        self.flag_vccs_publisher.publish(flag_vccs)

        print("DATA SENT ", self.counter)
        self.counter+=1
        

def main(args=None):
    rclpy.init(args=args)
    node = TRNode()

    cmd = Pose2D()
    cmd.x = 180.0
    cmd.y = 180.0
    node.neck_position_publisher.publish(cmd)
    print("INITIAL STATE")

    rclpy.spin(node)
    rclpy.shutdown()
 """