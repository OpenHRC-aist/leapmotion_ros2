import rclpy
from rclpy.node import Node
from hand_msgs.msg import Hands, Hand, Finger, Bone
import leap
from time import time
import numpy as np
from scipy.spatial.transform import Rotation

class HandsPublisher(Node):
    '''
    Leap Motionから手のすべての情報を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self):
        super().__init__('hands_publisher')
        self.hands_publisher = self.create_publisher(Hands, 'leap_hands', 10)
        self.listener = LeapMotionListener(self.handle_hand_data)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

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

    def handle_hand_data(self, hands):
        hands_msg = Hands()
        hands_msg.header.stamp = self.get_clock().now().to_msg()
        hands_msg.header.frame_id = "leap_link"

        for hand in hands:
            msg = Hand()
            msg.type = "Left" if str(hand.type) == "HandType.Left" else "Right"
            msg.position.x = -hand.palm.position.z * 1.0e-3
            msg.position.y = -hand.palm.position.x * 1.0e-3
            msg.position.z = hand.palm.position.y * 1.0e-3
            msg.velocity.x = -hand.palm.velocity.z * 1.0e-3
            msg.velocity.y = -hand.palm.velocity.x * 1.0e-3
            msg.velocity.z = hand.palm.velocity.y * 1.0e-3
            msg.normal.x = -hand.palm.normal.z
            msg.normal.y = -hand.palm.normal.x
            msg.normal.z = hand.palm.normal.y
            msg.orientation.x = -hand.palm.orientation.z
            msg.orientation.y = -hand.palm.orientation.x
            msg.orientation.z = hand.palm.orientation.y
            msg.orientation.w = hand.palm.orientation.w
            msg.grab_strength = hand.grab_strength
            msg.pinch_strength = hand.pinch_strength
            hands_msg.hands.append(msg)

            wrist = hand.arm.next_joint 

            for finger_idx, digit in enumerate(hand.digits):  
                finger_msg = Finger()
                finger_msg.id = finger_idx 

                finger_msg.tip_position.x = -(digit.bones[3].next_joint.z - wrist.z) * 1.0e-3
                finger_msg.tip_position.y = -(digit.bones[3].next_joint.x - wrist.x) * 1.0e-3
                finger_msg.tip_position.z =  (digit.bones[3].next_joint.y - wrist.y) * 1.0e-3

                direction = self.finger_direction(digit.distal)
                finger_msg.tip_direction.x = -direction[2]
                finger_msg.tip_direction.y = -direction[0]
                finger_msg.tip_direction.z =  direction[1]

                finger_msg.tip_orientation.x = -digit.distal.rotation.z
                finger_msg.tip_orientation.y = -digit.distal.rotation.x
                finger_msg.tip_orientation.z =  digit.distal.rotation.y
                finger_msg.tip_orientation.w =  digit.distal.rotation.w

                finger_msg.length = self.finger_length(digit) * 1.0e-3
                finger_msg.width  = digit.intermediate.width * 1.0e-3

                for bone_type, bone in enumerate(digit.bones): 
                    bone_msg = Bone()
                    bone_msg.type = bone_type 

                    bone_msg.prev_joint.x = -(bone.prev_joint.z - wrist.z) * 1.0e-3
                    bone_msg.prev_joint.y = -(bone.prev_joint.x - wrist.x) * 1.0e-3
                    bone_msg.prev_joint.z =  (bone.prev_joint.y - wrist.y) * 1.0e-3

                    bone_msg.next_joint.x = -(bone.next_joint.z - wrist.z) * 1.0e-3
                    bone_msg.next_joint.y = -(bone.next_joint.x - wrist.x) * 1.0e-3
                    bone_msg.next_joint.z =  (bone.next_joint.y - wrist.y) * 1.0e-3

                    bone_msg.rotation.x = -bone.rotation.z
                    bone_msg.rotation.y = -bone.rotation.x
                    bone_msg.rotation.z =  bone.rotation.y
                    bone_msg.rotation.w =  bone.rotation.w

                    bone_msg.width = bone.width * 1.0e-3

                    finger_msg.bones.append(bone_msg)

                msg.fingers.append(finger_msg)

        self.hands_publisher.publish(hands_msg)

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

def main(args=None):
    rclpy.init()
    hands_publisher = HandsPublisher()
    hands_publisher.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()