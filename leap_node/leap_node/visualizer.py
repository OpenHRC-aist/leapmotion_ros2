import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from hand_msgs.msg import Hand  # カスタムメッセージ型をインポート
from tf2_ros import TransformBroadcaster

class Visualizer(Node):
    '''
    Leap Motionの手の位置と姿勢をRViz2で可視化するクラス
    '''
    def __init__(self):
        super().__init__('visualizer')
        self.left_hand_subscriber = self.create_subscription(Hand, 'left_hand', self.handle_left_hand, 10)
        self.right_hand_subscriber = self.create_subscription(Hand, 'right_hand', self.handle_right_hand, 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定期的にTFフレームをブロードキャスト
        self.timer = self.create_timer(0.1, self.broadcast_leap_frame)

    def broadcast_leap_frame(self):
        '''
        TFフレーム "leap_frame" をブロードキャスト
        '''
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"  # 親フレーム
        transform.child_frame_id = "leap_frame"  # 子フレーム
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def handle_left_hand(self, msg):
        self.publish_marker(msg, "left_hand", [1.0, 0.0, 0.0])  # 赤色

    def handle_right_hand(self, msg):
        self.publish_marker(msg, "right_hand", [0.0, 0.0, 1.0])  # 青色

    def publish_marker(self, hand_msg, namespace, color):
        '''
        手の位置を円で、手の姿勢を法線ベクトルで表示
        '''
        # 手の位置を円で表示
        position_marker = Marker()
        position_marker.header.frame_id = "leap_frame"
        position_marker.header.stamp = self.get_clock().now().to_msg()
        position_marker.ns = namespace
        position_marker.id = 0 if namespace == "left_hand" else 1  # 左手と右手で異なるIDを設定
        position_marker.type = Marker.SPHERE
        position_marker.action = Marker.ADD
        position_marker.pose.position.x = hand_msg.position.x
        position_marker.pose.position.y = hand_msg.position.y
        position_marker.pose.position.z = hand_msg.position.z
        position_marker.pose.orientation.x = 0.0
        position_marker.pose.orientation.y = 0.0
        position_marker.pose.orientation.z = 0.0
        position_marker.pose.orientation.w = 1.0
        position_marker.scale.x = 0.05  # 円の直径
        position_marker.scale.y = 0.05
        position_marker.scale.z = 0.05
        position_marker.color.r = color[0]
        position_marker.color.g = color[1]
        position_marker.color.b = color[2]
        position_marker.color.a = 1.0
        self.marker_publisher.publish(position_marker)

        # 手の姿勢を法線ベクトル（矢印）で表示
        orientation_marker = Marker()
        orientation_marker.header.frame_id = "leap_frame"
        orientation_marker.header.stamp = self.get_clock().now().to_msg()
        orientation_marker.ns = namespace
        orientation_marker.id = 2 if namespace == "left_hand" else 3  # 左手と右手で異なるIDを設定
        orientation_marker.type = Marker.ARROW
        orientation_marker.action = Marker.ADD

        # 矢印の始点
        start_point = Point()
        start_point.x = hand_msg.position.x
        start_point.y = hand_msg.position.y
        start_point.z = hand_msg.position.z

        # 矢印の終点（法線ベクトルをスケールして加算）
        end_point = Point()
        end_point.x = hand_msg.position.x + hand_msg.normal.x * 0.1
        end_point.y = hand_msg.position.y + hand_msg.normal.y * 0.1
        end_point.z = hand_msg.position.z + hand_msg.normal.z * 0.1

        orientation_marker.points = [start_point, end_point]  # Point型のリストを渡す

        orientation_marker.scale.x = 0.02  # 矢印のシャフトの太さ
        orientation_marker.scale.y = 0.04  # 矢印のヘッドの太さ
        orientation_marker.scale.z = 0.0
        orientation_marker.color.r = color[0]
        orientation_marker.color.g = color[1]
        orientation_marker.color.b = color[2]
        orientation_marker.color.a = 1.0
        self.marker_publisher.publish(orientation_marker)

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()