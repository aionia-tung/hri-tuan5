"""Node dieu huong tu dong TurtleBot 4 qua chuoi goal bang Nav2.

Luong hoat dong:
  1. Doc danh sach goal (x, y, yaw) tu parameter (config/goals.yaml).
  2. Gui initial pose cho AMCL (topic /initialpose) va doi Nav2 active.
  3. Undock robot (action /undock cua Create 3).
  4. Lan luot gui tung goal qua action /navigate_to_pose, theo doi feedback,
     xu ly ket qua (SUCCEEDED / CANCELED / FAILED), dung lai de YOLO quan sat.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from visualization_msgs.msg import Marker, MarkerArray


def make_pose(navigator, x, y, yaw_deg):
    """Tao PoseStamped trong frame map tu (x, y, yaw[do])."""
    yaw = math.radians(yaw_deg)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    # Quaternion chi quay quanh truc z
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class GoalNavigator(TurtleBot4Navigator):

    def __init__(self):
        super().__init__()
        # TurtleBot4Navigator luon tao node ten 'basic_navigator'; launch file doi ten
        # thanh 'goal_navigator' (remap __node) de nap dung muc trong goals.yaml.
        # Luu y: BasicNavigator da dung self.status, nen trang thai goal dat la goal_status.
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('undock_first', True)
        self.declare_parameter('goal_names', ['Goal 1', 'Goal 2', 'Goal 3'])
        self.declare_parameter('goal_x', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_y', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_yaw', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_timeout_sec', 240.0)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('dwell_time_sec', 3.0)
        self.declare_parameter('return_home', False)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.initial = list(p('initial_pose'))
        self.set_initial = p('set_initial_pose')
        self.undock_first = p('undock_first')
        self.timeout = p('goal_timeout_sec')
        self.max_retries = p('max_retries')
        self.dwell = p('dwell_time_sec')

        names, xs, ys, yaws = p('goal_names'), p('goal_x'), p('goal_y'), p('goal_yaw')
        if not (len(names) == len(xs) == len(ys) == len(yaws)):
            raise ValueError('goal_names, goal_x, goal_y, goal_yaw phai cung do dai')
        self.goals = list(zip(names, xs, ys, yaws))
        if p('return_home'):
            self.goals.append(('Home', *self.initial))

        # Marker hien thi cac goal tren RViz2 (topic /goal_markers)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'goal_markers', qos)
        self.goal_status = ['pending'] * len(self.goals)
        self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        colors = {'pending': (1.0, 0.8, 0.0), 'active': (0.0, 0.6, 1.0),
                  'done': (0.0, 0.9, 0.2), 'failed': (1.0, 0.1, 0.1)}
        markers = MarkerArray()
        for i, (name, x, y, yaw) in enumerate(self.goals):
            r, g, b = colors[self.goal_status[i]]
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.ns, arrow.id, arrow.type = 'goals', i, Marker.ARROW
            arrow.pose = make_pose(self, x, y, yaw).pose
            arrow.pose.position.z = 0.05
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.12, 0.12
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.0
            text = Marker()
            text.header.frame_id = 'map'
            text.ns, text.id, text.type = 'goal_labels', i, Marker.TEXT_VIEW_FACING
            text.pose.position.x, text.pose.position.y = float(x), float(y)
            text.pose.position.z = 0.6
            text.scale.z = 0.35
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'G{i + 1}' if name != 'Home' else 'Home'
            markers.markers += [arrow, text]
        self.marker_pub.publish(markers)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def navigate(self, index):
        name, x, y, yaw = self.goals[index]
        for attempt in range(self.max_retries + 1):
            self.info(f'==> [{index + 1}/{len(self.goals)}] {name}: '
                      f'x={x:.2f}, y={y:.2f}, yaw={yaw:.0f} deg (lan thu {attempt + 1})')
            self.goal_status[index] = 'active'
            self.publish_markers()
            # goToPose() gui goal toi action server /navigate_to_pose (bt_navigator)
            self.goToPose(make_pose(self, x, y, yaw))

            start = self.get_clock().now()
            last_log = 0.0
            canceled = False
            while not self.isTaskComplete():
                feedback = self.getFeedback()
                if feedback is None:
                    continue
                elapsed = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
                if elapsed - last_log >= 2.0:
                    last_log = elapsed
                    eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    self.info(f'    con {feedback.distance_remaining:.2f} m, '
                              f'ETA {eta:.0f} s, recoveries={feedback.number_of_recoveries}')
                if not canceled and self.get_clock().now() - start > Duration(seconds=self.timeout):
                    self.warn(f'    Qua {self.timeout:.0f} s, huy goal.')
                    self.cancelTask()
                    canceled = True

            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.goal_status[index] = 'done'
                self.info(f'    DA DEN {name}.')
                return True
            self.warn(f'    {name} ket thuc voi ket qua {result.name}.')
        self.goal_status[index] = 'failed'
        return False

    def run(self):
        if self.set_initial:
            x, y, yaw = self.initial
            self.info(f'Dat initial pose cho AMCL: x={x}, y={y}, yaw={yaw} deg')
            self.setInitialPose(make_pose(self, x, y, yaw))
            # waitUntilNav2Active() tu publish lai /initialpose moi lan spin_once tra ve; voi
            # timer + /dock_status cua node nay, AMCL bi reset hang nghin lan va khong kip
            # publish /amcl_pose. Vi vay tu gui lai moi 2 s cho den khi AMCL tra loi.
            while not self.initial_pose_received:
                self.spin_for(2.0)
                if not self.initial_pose_received:
                    self.info('Chua nhan /amcl_pose, gui lai initial pose')
                    self._setInitialPose()

        # Doi amcl va bt_navigator chuyen sang trang thai active
        self.waitUntilNav2Active()

        if self.undock_first:
            for attempt in range(3):
                if not self.getDockedStatus():
                    break
                self.info(f'Robot dang o dock -> undock (lan {attempt + 1})')
                self.undock()
                self.spin_for(1.0)  # cap nhat /dock_status
            if self.getDockedStatus():
                self.warn('Undock khong thanh cong, van thu dieu huong.')

        results = []
        for i in range(len(self.goals)):
            results.append(self.navigate(i))
            self.publish_markers()
            if self.dwell > 0.0:
                self.info(f'    Dung {self.dwell:.0f} s de quan sat bang camera/YOLO')
                self.spin_for(self.dwell)

        ok = sum(results)
        self.info(f'Hoan thanh {ok}/{len(self.goals)} goal.')
        for (name, *_), success in zip(self.goals, results):
            self.info(f'  - {name}: {"THANH CONG" if success else "THAT BAI"}')


def main():
    rclpy.init()
    navigator = GoalNavigator()
    try:
        navigator.run()
        # Giu node de RViz2 van thay marker sau khi ket thuc
        navigator.spin_for(2.0)
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
