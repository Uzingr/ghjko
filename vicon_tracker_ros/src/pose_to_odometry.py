import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Odometry

class PoseToOdometry(Node):
    def __init__(self):
        super().__init__('pose_to_odometry')
        self.subscription = self.create_subscription(
            PoseStamped,
            'pose',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odometry', 10)

    def listener_callback(self, msg):
        odom_msg = Odometry()
        # Conversion logic here
        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
