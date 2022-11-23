import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import sys
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs.tf2_geometry_msgs as l
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

global_joint_states = None
lookup_Success = False
msg_ok = None
odom_ok = None
enter_odom = True
enter_lookup = True

class InverseKin(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.req = GetPositionIK.Request()
        self.point = Point()
        self.joint_ok = True
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self.create_subscription(TransformStamped, '/looked_up', self.lookup, 10)
        self.create_subscription(Odometry, '/Spot/odometry', self.odom_lookup, 10)
        self.srv = self.create_client(GetPositionIK,'/compute_ik')
        self.req.ik_request.group_name = 'spot_ur3e_arm'
        self.req.ik_request.robot_state.is_diff = True
        # self.timer = self.create_timer(0.1,self.lookup)
        # self.lookup()
        self.enter_joint_cb = True
        self.point_to_transform = PointStamped()
        self.new_point = PointStamped()

    def odom_lookup(self,odom_msg):
        global enter_odom
        if enter_odom:
            global odom_ok
            header = odom_msg.header
            pose = odom_msg.pose
            self.point_to_transform.header = header
            self.point_to_transform.point = pose.pose.position
            odom_ok = "not"
            

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
            # self.point.x = msg.transform.translation.x
            # self.point.y = msg.transform.translation.y
            # self.point.z = msg.transform.translation.z
            # self.req.ik_request.pose_stamped.pose.position = self.point 
            self.req.ik_request.pose_stamped.pose.position.x = 0.5 
            self.req.ik_request.pose_stamped.pose.position.y = 0.5 
            self.req.ik_request.pose_stamped.pose.position.z = 0.5 

            self.req.ik_request.pose_stamped.pose.orientation = msg.transform.rotation
            msg_ok = "not"
    
    # def point_gen(self):
    #     self.req.ik_request.pose_stamped.header.frame_id = 'base_link' 
    #     self.point.x = self.t.transform.translation.x
    #     self.point.y = self.t.transform.translation.y
    #     self.point.z = self.t.transform.translation.z
    #     self.req.ik_request.pose_stamped.pose.position = self.point
        # self.req.ik_request.pose_stamped.pose.position.x = 5.073
        # self.req.ik_request.pose_stamped.pose.position.y = 10.6
        # self.req.ik_request.pose_stamped.pose.position.z = 0.261
        # self.req.ik_request.pose_stamped.pose.orientation.x = 0.0
        # self.req.ik_request.pose_stamped.pose.orientation.y = -0.006
        # self.req.ik_request.pose_stamped.pose.orientation.z = -0.706
        # self.req.ik_request.pose_stamped.pose.orientation.w = 0.708


        # self.req.ik_request.pose_stamped.pose.orientation = self.t.transform.rotation

    def joint_states_cb(self, joint_state):
        if self.enter_joint_cb:
            print("Received JOint States")
            global global_joint_states
            global_joint_states = joint_state
            self.joint_state = joint_state
            self.req.ik_request.robot_state.joint_state = joint_state
            self.enter_joint_cb = False


    def request(self):
        # self.point_gen() #because transform.position is Vector3 whereas PoseStamped expects Point Type
        
        self.req.ik_request.avoid_collisions = False
        self.req.ik_request.timeout = Duration()
        self.req.ik_request.timeout.sec = 20
        # print(self.req)
        
        print("Calling Service")
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.future = self.srv.call_async(self.req)
        print("Sent Request")
        rclpy.spin_until_future_complete(self,self.future)
        print("Executed Service")
        return self.future.result()     
    def response_decode(self,response):
        print(response.solution.joint_state.name[0:5])
        print(response.solution.joint_state.position[0])
def main():
    global enter_odom,enter_lookup
    rclpy.init()

    ik_client = InverseKin()
    while odom_ok==None:
        rclpy.spin_once(ik_client)
    enter_odom = False
    while global_joint_states==None:
        rclpy.spin_once(ik_client)
    while msg_ok==None:
        rclpy.spin_once(ik_client)
    enter_lookup = False
    response = ik_client.request()
    print(response.error_code)
    ik_client.response_decode(response)
    ik_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()