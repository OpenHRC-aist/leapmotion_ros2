from hand_msgs.msg import Hands, Hand
from rclpy.node import Node
import rclpy
from ohrc_msgs.msg import State

class StateTopicPublisher(Node):
    def __init__(self):
        super().__init__('state_topic_publisher')
        self.hands_subscriber = self.create_subscription( 
            Hands,
            'leap_hands',
            self.hands_callback,
            10
        )

        self.state_publisher = self.create_publisher(State, '/cmd_state', 10)


    def hands_callback(self, msg):

        cmd = State()
        if len(msg.hands) != 2:
            cmd.enabled = False
            self.state_publisher.publish(cmd)
            return
        
        for hand in msg.hands:
            if hand.type == "Left":
                left_hand = hand
            elif hand.type == "Right":
                right_hand = hand

        cmd.pose.position.x =right_hand.position.x
        cmd.pose.position.y = right_hand.position.y
        cmd.pose.position.z = right_hand.position.z
        cmd.twist.linear.x = right_hand.velocity.x
        cmd.twist.linear.y = right_hand.velocity.y
        cmd.twist.linear.z = right_hand.velocity.z
        cmd.enabled = True if left_hand.grab_strength > 0.7 else False

        self.state_publisher.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = StateTopicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



