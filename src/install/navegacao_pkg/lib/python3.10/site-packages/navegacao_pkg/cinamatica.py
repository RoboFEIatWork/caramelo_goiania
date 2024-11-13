import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cinematica(Node):
    def __init__(self):
        super().__init__('cinematica')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)
        self.get_logger().info('Nó de publisher para /cmd_vel iniciado.')

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.1  # Velocidade linear em x
        msg.angular.z = 0.0  # Velocidade angular em z

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = cinematica()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
