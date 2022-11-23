import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point
from copy import deepcopy
from nav_msgs.msg import Odometry
import tf2_geometry_msgs.tf2_geometry_msgs as l

global_joint_states = None
lookup_Success = False
msg_ok = None
enter_lookup = True
odom_ok = None
enter_odom = True

class Combined(Node):

    def __init__(self):
        super().__init__('service_and_action')
        #for action
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True
        #for service
        self.req = GetPositionIK.Request()
        self.point = Point()
        self.joint_ok = True
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self.create_subscription(Odometry, '/Spot/odometry', self.odom_lookup, 10)
        self.create_subscription(TransformStamped, '/looked_up', self.lookup, 10)
        self.srv = self.create_client(GetPositionIK,'/compute_ik')
        self.req.ik_request.group_name = 'spot_ur3e_arm'
        self.req.ik_request.robot_state.is_diff = True
        self.enter_joint_cb = True
        self.point_to_transform = PointStamped()
        self.new_point = PointStamped()

        self.jc = JointConstraint()
        self.jc.tolerance_above = 0.0001
        self.jc.tolerance_below = 0.0001
        self.jc.weight = 1.0

        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_ur3e_arm'
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = False
        self.planning_options.look_around_attempts = 0
        self.planning_options.max_safe_execution_cost = 0.
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 10
        self.planning_options.replan_delay = 0.1

        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def odom_lookup(self,odom_msg):
        global enter_odom
        if enter_odom:
            global odom_ok
            header = odom_msg.header
            pose = odom_msg.pose
            self.point_to_transform.header = header
            self.point_to_transform.point = pose.pose.position
            odom_ok = "not"

    def build_constraint(self,response):
        joints = {}
        # print(response)
        joints['shoulder_pan_joint'] = response.solution.joint_state.position[0]
        joints['shoulder_lift_joint'] = response.solution.joint_state.position[1]
        joints['elbow_joint'] = response.solution.joint_state.position[2]
        joints['wrist_1_joint'] = response.solution.joint_state.position[3]
        joints['wrist_2_joint'] = response.solution.joint_state.position[4]
        joints['wrist_3_joint'] = response.solution.joint_state.position[5]

        constraints = Constraints()
        for (joint, angle) in joints.items():
            self.jc.joint_name = joint
            self.jc.position = angle
            constraints.joint_constraints.append(deepcopy(self.jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.send_goal()

    def send_goal(self):
        self.motion_plan_request.start_state.joint_state = self.joint_state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(str(future))

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))
        self.destroy_node()
        rclpy.shutdown()

    def lookup(self,msg):
        global enter_lookup
        if enter_lookup:
            global msg_ok
            # self.msg2 = msg
            self.new_point = l.do_transform_point(self.point_to_transform,msg)
            x = self.new_point.point.x-self.point_to_transform.point.x
            y = self.new_point.point.y-self.point_to_transform.point.y
            z = self.new_point.point.z-self.point_to_transform.point.z
            print(f"X,Y AND Z are {x},{y},{z}")
            self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

            self.req.ik_request.pose_stamped.pose.position.x = 0.2
            self.req.ik_request.pose_stamped.pose.position.y = 0.05 
            self.req.ik_request.pose_stamped.pose.position.z = 0.5
            self.req.ik_request.pose_stamped.pose.orientation = msg.transform.rotation
            msg_ok = "not"

    def joint_states_cb(self, joint_state):
        if self.enter_joint_cb:
            print("Received JOint States")
            global global_joint_states
            global_joint_states = joint_state
            self.joint_state = joint_state
            self.req.ik_request.robot_state.joint_state = joint_state
            self.enter_joint_cb = False

    def request(self):
        self.req.ik_request.avoid_collisions = False
        self.req.ik_request.timeout = Duration()
        self.req.ik_request.timeout.sec = 20
        
        print("Calling Service")
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.future = self.srv.call_async(self.req)
        print("Sent Request")
        rclpy.spin_until_future_complete(self,self.future)
        print("Executed Service")
        return self.future.result()  

def main():
    global enter_odom,enter_lookup
    rclpy.init()

    mixed = Combined()
    while odom_ok==None:
        rclpy.spin_once(mixed)
    enter_odom = False
    while global_joint_states==None:
        rclpy.spin_once(mixed)
    while msg_ok==None:
        rclpy.spin_once(mixed)
    enter_lookup = False
    response = mixed.request()
    # print(response)
    mixed.build_constraint(response)