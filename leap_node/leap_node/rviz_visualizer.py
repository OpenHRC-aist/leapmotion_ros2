import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from hand_msgs.msg import Hand  # カスタムメッセージ型をインポート
import subprocess

class RvizVisualizer(Node):
    '''
    Leap Motionの手の位置と姿勢をPoseArrayでRViz2に可視化するクラス
    '''
    def __init__(self):
        super().__init__('rviz_visualizer')
        self.pose_array_publisher = self.create_publisher(PoseArray, 'hand_poses', 10)
        self.left_hand_subscriber = self.create_subscription(Hand, 'left_hand', self.handle_left_hand, 10)
        self.right_hand_subscriber = self.create_subscription(Hand, 'right_hand', self.handle_right_hand, 10)

        # 左手と右手のデータを保持
        self.left_hand_pose = None
        self.right_hand_pose = None

        # 定期的にPoseArrayを送信
        self.timer = self.create_timer(0.1, self.publish_pose_array)

        # RVizを自動起動
        self.start_rviz()

    def start_rviz(self):
        '''
        RVizを自動的に起動
        '''
        rviz_config_path = 'leap_node/rviz/leapmotion.rviz'
        subprocess.Popen(['rviz2', '-d', rviz_config_path])

    def handle_left_hand(self, msg):
        '''
        左手のデータを処理
        '''
        self.left_hand_pose = self.create_pose_from_hand(msg)

    def handle_right_hand(self, msg):
        '''
        右手のデータを処理
        '''
        self.right_hand_pose = self.create_pose_from_hand(msg)

    def create_pose_from_hand(self, hand_msg):
        '''
        HandメッセージからPoseを作成
        '''
        pose = Pose()
        pose.position.x = hand_msg.position.x
        pose.position.y = hand_msg.position.y
        pose.position.z = hand_msg.position.z
        pose.orientation.x = hand_msg.orientation.x
        pose.orientation.y = hand_msg.orientation.y
        pose.orientation.z = hand_msg.orientation.z
        pose.orientation.w = hand_msg.orientation.w
        return pose

    def publish_pose_array(self):
        '''
        PoseArrayを送信
        '''
        pose_array = PoseArray()
        pose_array.header.frame_id = "leap_frame"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # 左手と右手のPoseを追加
        if self.left_hand_pose:
            pose_array.poses.append(self.left_hand_pose)
        if self.right_hand_pose:
            pose_array.poses.append(self.right_hand_pose)

        # PoseArrayを送信
        self.pose_array_publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()