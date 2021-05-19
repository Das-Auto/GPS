import rclpy
from rclpy.node import Node

from message_object.msg import MessageObject    

steps = []

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(MessageObject, 'GPStopic',self.listener_callback, 10)  

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        steps.append(msg)
        self.get_logger().info('listener : maneuver: "%d" ' % steps[-1].next_maneuver )
        self.get_logger().info('listener : distance remaining in step %d meters' % steps[-1].distance_remaining_in_step )    
        self.get_logger().info('listener : destination lng: "%.6f" ' % steps[-1].end_location_lat  )
        self.get_logger().info('listener : destination lat: "%.6f" ' % steps[-1].end_location_lng )      


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()