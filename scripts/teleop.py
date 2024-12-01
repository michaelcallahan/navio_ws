import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'command', 10)
        self.get_logger().info('Teleop node initialized. Use WASD keys to move the quadcopter, Q/E to change yaw.')
        self.run_teleop()

    def run_teleop(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl-C to quit
                    break
                self.publish_command(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def publish_command(self, key):
        twist = Twist()
        if key == 'w':
            twist.linear.x = 1.0
        elif key == 's':
            twist.linear.x = -1.0
        elif key == 'a':
            twist.linear.y = 1.0
        elif key == 'd':
            twist.linear.y = -1.0
        elif key == 'q':
            twist.angular.z = 1.0
        elif key == 'e':
            twist.angular.z = -1.0
        elif key == 'x':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        else:
            return
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
