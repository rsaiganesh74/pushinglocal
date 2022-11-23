import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import sys

class Lookup(Node):
    def __init__(self):
        super().__init__('lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TransformStamped,'/looked_up',10)
        self.timer = self.create_timer(0.1,self.lookup_cb)
        
    def lookup_cb(self):
        global lookup_Success
        try:
            self.t = self.tf_buffer.lookup_transform('base_link','can5',rclpy.time.Time())
            # print('Received Lookup, exiting')
            self.pub.publish(self.t)
        except:
            print("Not getting Lookup")

def main():
    rclpy.init()

    lookup = Lookup()

    try:
        rclpy.spin(lookup)
    except KeyboardInterrupt:

        lookup.destroy_node()
        rclpy.shutdown()
        sys.exit()


if __name__ == '__main__':
    main()