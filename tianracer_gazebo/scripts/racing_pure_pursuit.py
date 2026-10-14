#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线时间最优赛车线 + Pure Pursuit 纯跟踪（计时赛方案一）。

- 赛道固定：离线生成的赛车线与速度剖面（90% 抓地）在 track_file(CSV: x,y,yaw,velocity)。
- 不经过 move_base / 不切换 goal：沿一条闭合轨迹连续跟踪，避免规划失败与 goal 切换停顿。
- 直接发布 AckermannDriveStamped 到 ackermann_cmd_stamped（servo_commands 接收）。
- 坐标：odom 直接给出 Gazebo 世界坐标，与轨迹同系。
裁判机制不做任何改动。
"""
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion


def wrap(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a


class RacingPurePursuit:
    def __init__(self):
        track_file = rospy.get_param('~track_file', '')
        rospy.loginfo('racing pure pursuit, track file: %s', track_file)
        d = np.loadtxt(track_file, delimiter=',', skiprows=1)
        self.tx, self.ty, self.tv = d[:, 0], d[:, 1], d[:, 3]
        self.n = len(self.tx)
        # per-point forward arc length
        nx = (np.arange(self.n)+1) % self.n
        self.ds = np.hypot(self.tx[nx]-self.tx, self.ty[nx]-self.ty)
        self.lap_len = float(self.ds.sum())

        self.wheelbase = float(rospy.get_param('~wheelbase', 0.26))
        self.laps_target = int(rospy.get_param('~laps', 5))
        self.acc = float(rospy.get_param('~acc_limit', 7.0))
        self.brake = 10.0
        self.max_steer = 0.55
        self.ld_k = 0.28       # lookahead = base + k*v
        self.ld_base = 0.40

        self.x = self.y = self.yaw = None
        self.idx_u = 0          # unwrapped progress index (monotonic)
        self.initialized = False
        self.cmd_v = 0.0
        self.finished = False

        self.pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        q = p.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

    def at(self, iu):
        i = int(iu) % self.n
        return i

    def find_closest(self):
        if not self.initialized:
            dd = (self.tx-self.x)**2+(self.ty-self.y)**2
            self.idx_u = int(np.argmin(dd))
            self.initialized = True
            return
        # search forward window for nearest (monotonic, no going back)
        best_off, best_d = 0, None
        for off in range(0, 70):
            i = self.at(self.idx_u+off)
            dd = (self.tx[i]-self.x)**2+(self.ty[i]-self.y)**2
            if best_d is None or dd < best_d:
                best_d, best_off = dd, off
        self.idx_u += best_off

    def control(self, dt):
        self.find_closest()
        # lookahead distance & target index
        ld = self.ld_base + self.ld_k*max(self.cmd_v, 0.0)
        acc, iu = 0.0, self.idx_u
        li = self.at(iu)
        while acc < ld:
            acc += self.ds[self.at(iu)]
            iu += 1
            li = self.at(iu)
        # pure pursuit steering
        dx = self.tx[li]-self.x
        dy = self.ty[li]-self.y
        alpha = wrap(math.atan2(dy, dx)-self.yaw)
        Ld = max(math.hypot(dx, dy), 1e-3)
        delta = math.atan2(2*self.wheelbase*math.sin(alpha), Ld)
        delta = max(-self.max_steer, min(self.max_steer, delta))

        # target speed = minimum profile speed between closest and lookahead (early braking)
        v_target = self.tv[self.at(self.idx_u)]
        for off in range(0, max(int(iu-self.idx_u), 1)+1):
            v_target = min(v_target, self.tv[self.at(self.idx_u+off)])

        # progress / lap counting
        total_s = (self.idx_u // self.n)*self.lap_len + \
            float(np.sum(self.ds[: self.at(self.idx_u)]))
        lap = int(total_s/self.lap_len)

        # finish: crossed the lap line after target laps
        over = total_s - self.laps_target*self.lap_len
        if lap >= self.laps_target and over > 0.3:
            self.finished = True
        if self.finished:
            v_target = 0.0

        # speed rate limiting (acc / brake)
        if v_target > self.cmd_v:
            self.cmd_v = min(v_target, self.cmd_v+self.acc*dt)
        else:
            self.cmd_v = max(v_target, self.cmd_v-self.brake*dt)

        self.publish(self.cmd_v, delta)
        rospy.loginfo('lap %d/%d  total_s %.1f  v %.2f (tgt %.2f)  steer %.2f',
                      min(lap, self.laps_target), self.laps_target, total_s,
                      self.cmd_v, v_target, delta)

    def publish(self, speed, steer):
        m = AckermannDriveStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'base_link'
        m.drive.speed = speed
        m.drive.steering_angle = steer
        self.pub.publish(m)

    def spin(self):
        rate = rospy.Rate(30)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max((now-last).to_sec(), 1e-3)
            last = now
            if self.x is not None:
                self.control(min(dt, 0.1))
                if self.finished and self.cmd_v <= 0.02:
                    self.publish(0.0, 0.0)
                    rospy.loginfo('racing finished after %d laps, vehicle stopped', self.laps_target)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('racing_pure_pursuit')
    try:
        RacingPurePursuit().spin()
    except rospy.ROSInterruptException:
        pass
