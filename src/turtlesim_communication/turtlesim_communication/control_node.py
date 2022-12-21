import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleBotControl(Node):

    def __init__(self):
        super().__init__('turtle_bot_control')
        self.publisher_ = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10)
        self.subscriber = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.subscriber_callback, 10)
        self.subscriber # to prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing to cmd_vel')
        
    def subscriber_callback(self, msg):
        self.get_logger().info("x: %.2f, y: %.2f" % (msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)

    control_node = TurtleBotControl()

    rclpy.spin(control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
