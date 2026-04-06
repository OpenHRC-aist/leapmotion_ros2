import rclpy
from rclpy.node import Node
from leap_msgs.msg import Finger  # カスタムメッセージ型をインポート
from geometry_msgs.msg import Point
import leap
import time
import curses

class FingerDetector(Node):
    '''
    Leap Motionから手の指の関節情報を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self, stdscr):
        super().__init__('finger_detector')
        self.left_hand_publisher = self.create_publisher(Finger, 'left_hand_fingers', 10)
        self.right_hand_publisher = self.create_publisher(Finger, 'right_hand_fingers', 10)
        self.stdscr = stdscr
        self.listener = LeapMotionListener(self.handle_finger_data)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

    def _to_meters(self, leap_pos):
        return Point(
            x=-leap_pos.z * 0.001,  # Z -> -X 
            y=-leap_pos.x * 0.001,  # X -> -Y
            z=leap_pos.y * 0.001    # Y -> Z
        )

    def handle_finger_data(self, hands):
        # 検出された手を処理
        self.stdscr.clear()
        for hand in hands:
            for digit in hand.digits:
                
                finger_msg = Finger()
                finger_msg.id = digit.finger_id
                tip_point = self._to_meters(digit.distal.next_joint)
                finger_msg.tip_position = tip_point
                
                if str(hand.type) == "HandType.Left":
                    self.left_hand_publisher.publish(finger_msg)
                    self.stdscr.addstr(f"Left Hand Finger ID {finger_msg.id}:\n Tip Position: x={finger_msg.tip_position.x:.2f}, y={finger_msg.tip_position.y:.2f}, z={finger_msg.tip_position.z:.2f}\n")
                elif str(hand.type) == "HandType.Right":
                    self.right_hand_publisher.publish(finger_msg)
                    self.stdscr.addstr(f"Right Hand Finger ID {finger_msg.id}:\n Tip Position: x={finger_msg.tip_position.x:.2f}, y={finger_msg.tip_position.y:.2f}, z={finger_msg.tip_position.z:.2f}\n")
        self.stdscr.refresh()

    def run(self):
        with self.connection.open():
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while rclpy.ok():
                rclpy.spin_once(self)
                time.sleep(1)

class LeapMotionListener(leap.Listener):
    '''
    Leap Motionのイベント処理を行うクラス
    '''
    def __init__(self, callback):
        super().__init__()
        self.callback = callback  # コールバック関数を保持

    def on_tracking_event(self, event):
        self.callback(event.hands)  # 検出された手をコールバック関数に渡す

def curses_main(stdscr):
    rclpy.init()
    node = FingerDetector(stdscr)
    node.run()
    rclpy.shutdown()

def main(args=None):
    """ rclpy.init()
    node = FingerDetector()
    node.run()
    rclpy.shutdown() """
    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()