import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class PoseToOdometry(Node):
    def __init__(self):
        super().__init__('pose_to_odometry')

        # Iscriviti al topic 'pose'
        self.pose_subscription = self.create_subscription(
            Pose,
            'hagrid/pose',
            self.pose_callback,
            10
        )

        # Publisher per il topic 'mocap_odometry'
        self.odometry_publisher = self.create_publisher(Odometry, 'mocap_odometry', 10)

    def pose_callback(self, pose_msg):
        # Crea un messaggio Odometry
        odom_msg = Odometry()

        # Riempie il messaggio Odometry con i dati del messaggio Pose
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = pose_msg

        # Pubblica il messaggio Odometry
        self.odometry_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
