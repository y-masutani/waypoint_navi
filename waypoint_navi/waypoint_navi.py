import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose


class WayPointNavi(Node):
    def __init__(self):
        super().__init__('waypoint_navi')  # ノードの初期化
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

    def send_goal(self, goal2d):
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('アクションサーバが起動するまで待ちます．')
        self.get_logger().info(f'({goal2d[0]}, {goal2d[1]}, {goal2d[2]})へ行きます．')

        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = self.make_pose(goal2d)

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('ゴールは断られました．')
            return

        self.get_logger().info('ゴールは受け付けられました．')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('ゴールに着きました．')
        elif future.result().status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('ゴールに着けませんでした．')

    def feedback_callback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().info(f'残り{self.feedback.distance_remaining:.2f}[m]')

    def make_pose(self, pose2d):
        x, y, yaw = pose2d
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def publish_initial_pose(self, pose2d):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.make_pose(pose2d)
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info('初期ポーズをパブリッシュします．')
        self.initial_pose_pub.publish(msg)

    def do_navigation(self, goal2d):
        self.publish_initial_pose([0.0, 0.0, 0.0])   # 初期位置の設定
        self.send_goal(goal2d)  # ゴールの送信


def main(args=None):
    rclpy.init(args=args)
    node = WayPointNavi()
    x = float(input('ゴールx座標: '))
    y = float(input('ゴールy座標: '))
    yaw = float(input('ゴール方向: '))
    node.do_navigation([x, y, yaw])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
