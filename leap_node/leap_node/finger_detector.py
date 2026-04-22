import rclpy
from rclpy.node import Node
from leap_msgs.msg import Finger # カスタムメッセージ型をインポート
from geometry_msgs.msg import Point, Vector3
import leap
import time
import curses
import numpy as np
from scipy.spatial.transform import Rotation

class FingerDetector(Node):
    '''
    Leap Motionから手の指の関節情報を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self, stdscr):
        super().__init__('finger_detector')
        self.left_fingers_publisher = self.create_publisher(Finger, 'left_hand_fingers', 10)
        self.right_fingers_publisher = self.create_publisher(Finger, 'right_hand_fingers', 10)
        self.stdscr = stdscr
        self.listener = LeapMotionListener(self.handle_finger_data)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

    def position_relative_to_wrist(self, hand, bone):
        wrist_position = np.array([hand.arm.next_joint.x, hand.arm.next_joint.y, hand.arm.next_joint.z])
        q_wrist_position = Rotation.from_quat([hand.palm.orientation.x, hand.palm.orientation.y, hand.palm.orientation.z, hand.palm.orientation.w])
        tip_position = np.array([bone.next_joint.x, bone.next_joint.y, bone.next_joint.z])
        tip_relative = q_wrist_position.inv().apply(tip_position - wrist_position)
        return tip_relative

    def bone_length(self, bone):
        tip = bone.next_joint
        base = bone.prev_joint
        return np.linalg.norm([tip.x - base.x, tip.y - base.y, tip.z - base.z])
    
    def finger_length(self, digit):
        return sum(self.bone_length(digit.bones[i]) for i in range(4))

    def finger_direction(self, bone):
        q = Rotation.from_quat([bone.rotation.x, bone.rotation.y, bone.rotation.z, bone.rotation.w])
        direction = q.apply([0, 0, -1])
        return direction

    def handle_finger_data(self, hands):
        # 検出された手を処理
        self.stdscr.clear()
        for hand in hands:

            for digit in hand.digits:
                finger_msg = Finger()
                finger_msg.id = digit.finger_id
                rel_position = self.position_relative_to_wrist(hand, digit.distal)
                finger_msg.tip_position.x = -rel_position[2] * 1.0e-3
                finger_msg.tip_position.y = -rel_position[0] * 1.0e-3
                finger_msg.tip_position.z = rel_position[1] * 1.0e-3
                direction = self.finger_direction(digit.distal)
                finger_msg.tip_direction.x = -direction[2]
                finger_msg.tip_direction.y = -direction[0]
                finger_msg.tip_direction.z = direction[1]
                finger_msg.tip_orientation.x = -digit.distal.rotation.z
                finger_msg.tip_orientation.y = -digit.distal.rotation.x
                finger_msg.tip_orientation.z = digit.distal.rotation.y
                finger_msg.tip_orientation.w = digit.distal.rotation.w
                finger_msg.length = self.finger_length(digit) * 1.0e-3
                finger_msg.width = digit.intermediate.width * 1.0e-3

                if str(hand.type) == "HandType.Left":
                    self.left_fingers_publisher.publish(finger_msg)
                    try:
                        self.stdscr.addstr(
                            f"Left Hand Finger ID {finger_msg.id}:\n"
                            f" Tip Position: x={finger_msg.tip_position.x:.2f},"
                            f" y={finger_msg.tip_position.y:.2f},"
                            f" z={finger_msg.tip_position.z:.2f}\n"
                            f" Tip Direction: x={finger_msg.tip_direction.x:.2f},"
                            f" y={finger_msg.tip_direction.y:.2f},"
                            f" z={finger_msg.tip_direction.z:.2f}\n"
                            f" Length: {finger_msg.length:.2f}\n"
                            f" Width: {finger_msg.width:.2f}\n"
                        )
                    except curses.error:
                        pass
                        
                elif str(hand.type) == "HandType.Right":
                    self.right_fingers_publisher.publish(finger_msg)
                    try:
                        self.stdscr.addstr(
                            f"Right Hand Finger ID {finger_msg.id}:\n"
                            f" Tip Position: x={finger_msg.tip_position.x:.2f},"
                            f" y={finger_msg.tip_position.y:.2f},"
                            f" z={finger_msg.tip_position.z:.2f}\n"
                            f" Tip Direction: x={finger_msg.tip_direction.x:.2f},"
                            f" y={finger_msg.tip_direction.y:.2f},"
                            f" z={finger_msg.tip_direction.z:.2f}\n"
                            f" Length: {finger_msg.length:.2f}\n"
                            f" Width: {finger_msg.width:.2f}\n"
                        )
                    except curses.error:
                        pass
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