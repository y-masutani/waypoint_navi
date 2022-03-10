import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

import rclpy

from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy)
from rclpy.qos import QoSProfile


class Navigator(Node):
    def __init__(self):
        super().__init__(node_name='navigator')
        self.timeout = 180.0  # [s]

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback,
            amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

    def set_initial_pose(self, pose2d):
        self.initial_pose_received = False
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.pose = self.make_pose(pose2d)
        self.publish_initial_pose()

    def send_goal(self, goal2d):
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('アクションサーバが起動するまで待ちます．')

        self.get_logger().info(f'({goal2d[0]}, {goal2d[1]}, {goal2d[2]})へ行きます．')

        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = self.make_pose(goal2d)
        goal_msg.pose = pose_stamped

        self.feedback = None
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('ゴールは断られました．')
            return False
        self.get_logger().info('ゴールは受け付けられました．')
        self.result_future = self.goal_handle.get_result_async()
        count = 0
        while not self.result_future.result():
            if self.feedback and count % 5 == 0:
                self.get_logger().info(
                    f'残り{self.feedback.distance_remaining:.2f}[m]')
                if (Duration.from_msg(self.feedback.navigation_time)
                        > Duration(seconds=self.timeout)):
                    self.get_logger().info(
                        f'{self.timeout}[s]経過したので取り消します．')
                    self.cancel_nav()
            rclpy.spin_until_future_complete(
                self, self.result_future, timeout_sec=0.10)
            count += 1

        status = self.result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('ゴールに着きました．')
            return True
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('ゴールが中断されました．')
            return False
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('ゴールが取り消されました．')
            return False
        else:
            self.get_logger().info('結果が不明です．')
            return False

    def feedback_callback(self, msg):
        self.feedback = msg.feedback

    def cancel_nav(self):
        self.get_logger().info('現在のゴールを取り消します．')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

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

    def wait_for_nav2(self):
        self.wait_for_node('amcl')
        self.wait_for_initial_pose()
        self.wait_for_node('bt_navigator')
        self.get_logger().info('Nav2が使えるようになりました')
        return

    def wait_for_node(self, node_name):
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{node_service}のサービス無効．待機します...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
            time.sleep(2)
        return

    def wait_for_initial_pose(self):
        while not self.initial_pose_received:
            self.get_logger().info('初期ポーズ設定')
            self.publish_initial_pose()
            self.get_logger().info('amclのポーズ受信待ち')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def amcl_pose_callback(self, msg):
        self.initial_pose_received = True
        return

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.get_logger().info('初期ポーズをパブリッシュ')
        self.initial_pose_pub.publish(msg)
        return


def main(args=None):
    rclpy.init(args=args)

    node = Navigator()
    try:
        node.set_initial_pose([0.0, 0.0, 0.0])
        node.wait_for_nav2()

        while True:
            x = float(input('ゴールx座標: '))
            y = float(input('ゴールy座標: '))
            yaw = float(input('ゴール方向: '))
            node.send_goal([x, y, yaw])
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
