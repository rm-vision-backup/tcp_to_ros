#!/usr/bin/env python3
"""Freeze Group A global origin/reference heading and gate task readiness."""
import collections
import math
import threading

import rospy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion

from tcp_to_ros.calibration import (GlobalPoint, circular_mean, headings_within,
                                    median_global, validate_calibration_config)
from tcp_to_ros.identity import IdentityMap


class GcsACalibrationNode:
    def __init__(self):
        self._lock = threading.RLock()
        # 执行层 UAVn 身份（implementation_plan_26091101 §3.1）：calibration_uavs
        # 决定参与采样/冻结的集合；all_uavs 仅用于 gp_origin_confirmed 记账与
        # 逐机状态展示，不作为 ready 的全量阻塞门槛。~identity_map 仅作兼容
        # （UAVn→topic 前缀），默认恒等；~identity_file 用于启动校验。
        self.calibration_uavs = tuple(rospy.get_param(
            "~calibration_uavs", ["UAV%d" % i for i in range(1, 7)]))
        self.all_uavs = tuple(rospy.get_param(
            "~all_uavs", ["UAV%d" % i for i in range(1, 17)]))
        self.identity = dict(rospy.get_param("~identity_map", {}) or {})
        identity_file = str(rospy.get_param("~identity_file", "") or "")
        identities = IdentityMap.load(identity_file).uavs if identity_file else None
        self.calibration_uavs, self.all_uavs = validate_calibration_config(
            self.calibration_uavs, self.all_uavs, identities)
        rospy.loginfo("calibration set=%s all_uavs=%d identity_file=%s",
                      ",".join(self.calibration_uavs), len(self.all_uavs),
                      identity_file or "(none)")
        self.sample_window_s = float(rospy.get_param("~sample_window_s", 2.0))
        self.min_samples = int(rospy.get_param("~min_samples_per_uav", 5))
        self.sample_max_age_s = float(rospy.get_param("~sample_max_age_s", 2.0))
        self.max_heading_error = math.radians(float(rospy.get_param("~max_heading_error_deg", 30.0)))
        self.positions = {uav: collections.deque() for uav in self.calibration_uavs}
        self.headings = {uav: collections.deque() for uav in self.calibration_uavs}
        self.states = {}
        self.confirmed = {uav: False for uav in self.all_uavs}
        self.frozen_origin = None
        self.frozen_heading = None
        self.ready = False
        self.origin_broadcast_period_s = 1.0 / float(
            rospy.get_param("~origin_broadcast_hz", 1.0))
        if self.origin_broadcast_period_s <= 0.0:
            raise ValueError("origin_broadcast_hz must be positive")
        self._last_origin_broadcast_s = 0.0
        self.origin_pub = rospy.Publisher("/group_a/gp_origin", GeoPointStamped, queue_size=1, latch=True)
        self.heading_pub = rospy.Publisher("/group_a/reference_heading", Float64, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher("/group_a/calibration_ready", Bool, queue_size=1, latch=True)
        self.ready_pub.publish(Bool(False))
        for uav in self.all_uavs:
            target = self.identity.get(uav, uav)
            rospy.Subscriber("/%s/mavros/state" % target, State, self.on_state,
                             callback_args=uav, queue_size=10)
            rospy.Subscriber("/%s/gp_origin_confirmed" % target, Bool, self.on_confirmed,
                             callback_args=uav, queue_size=1)
            if uav in self.positions:
                rospy.Subscriber("/%s/mavros/global_position/global" % target, NavSatFix,
                                 self.on_global, callback_args=uav, queue_size=20)
                rospy.Subscriber("/%s/mavros/local_position/pose" % target, PoseStamped,
                                 self.on_pose, callback_args=uav, queue_size=20)
        rospy.Timer(rospy.Duration(0.1), self.on_timer)

    def _append(self, table, uav, value):
        now = rospy.Time.now().to_sec()
        queue = table[uav]
        queue.append((now, value))
        cutoff = now - self.sample_window_s
        while queue and queue[0][0] < cutoff:
            queue.popleft()

    def on_global(self, msg, uav):
        if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
            return
        with self._lock:
            if self.frozen_origin is None:
                self._append(self.positions, uav, GlobalPoint(msg.latitude, msg.longitude, msg.altitude))

    def on_pose(self, msg, uav):
        q = msg.pose.orientation
        yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        with self._lock:
            if self.frozen_origin is None:
                self._append(self.headings, uav, yaw)

    def on_state(self, msg, uav):
        with self._lock:
            self.states[uav] = (rospy.Time.now().to_sec(), bool(msg.connected), bool(msg.armed))

    def on_confirmed(self, msg, uav):
        with self._lock:
            self.confirmed[uav] = bool(msg.data)

    def _representatives(self, table, now):
        result = {}
        for uav in self.calibration_uavs:
            values = [value for stamp, value in table[uav] if now - stamp <= self.sample_window_s]
            if len(values) < self.min_samples or not table[uav] or now - table[uav][-1][0] > self.sample_max_age_s:
                return None
            if table[uav][-1][0] - table[uav][0][0] < self.sample_window_s * 0.95:
                return None
            result[uav] = values
        return result

    def _try_freeze(self, now):
        for uav in self.calibration_uavs:
            state = self.states.get(uav)
            if state is None or now - state[0] > self.sample_max_age_s or not state[1] or state[2]:
                return
        positions = self._representatives(self.positions, now)
        headings = self._representatives(self.headings, now)
        if positions is None or headings is None:
            return
        per_uav_positions = [median_global(positions[uav]) for uav in self.calibration_uavs]
        per_uav_headings = [circular_mean(headings[uav]) for uav in self.calibration_uavs]
        reference = circular_mean(per_uav_headings)
        if not headings_within(per_uav_headings, reference, self.max_heading_error):
            rospy.logerr_throttle(2.0, "calibration blocked: heading spread in %s exceeds %.0f degrees",
                                  ",".join(self.calibration_uavs),
                                  math.degrees(self.max_heading_error))
            return
        self.frozen_origin = median_global(per_uav_positions)
        self.frozen_heading = reference
        self._publish_origin(now)
        self.heading_pub.publish(Float64(reference))
        rospy.set_param("/group_a/frozen_reference_heading", reference)
        rospy.loginfo("calibration frozen from %s (heading gate %.0fdeg) "
                      "origin=(%.9f,%.9f,%.3f) heading=%.3f rad",
                      ",".join(self.calibration_uavs),
                      math.degrees(self.max_heading_error),
                      self.frozen_origin.latitude, self.frozen_origin.longitude,
                      self.frozen_origin.altitude, reference)

    def _publish_origin(self, now):
        """Broadcast the frozen origin so late/restarted receivers can recover."""
        if (self._last_origin_broadcast_s > 0.0
                and now - self._last_origin_broadcast_s < self.origin_broadcast_period_s):
            return
        msg = GeoPointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "earth"
        msg.position.latitude = self.frozen_origin.latitude
        msg.position.longitude = self.frozen_origin.longitude
        msg.position.altitude = self.frozen_origin.altitude
        self.origin_pub.publish(msg)
        self._last_origin_broadcast_s = now

    def on_timer(self, _event):
        with self._lock:
            now = rospy.Time.now().to_sec()
            if self.frozen_origin is None:
                self._try_freeze(now)
                return
            self._publish_origin(now)
            if not self.ready:
                confirmed = [uav for uav in self.calibration_uavs
                             if self.confirmed[uav]]
                if len(confirmed) == len(self.calibration_uavs):
                    self.ready = True
                    self.ready_pub.publish(Bool(True))
                    rospy.set_param("/group_a/calibration_ready", True)
                    rospy.loginfo("Group A calibration ready: %d/%d required UAVs "
                                  "confirmed (%s); tracking %d known UAVs",
                                  len(confirmed), len(self.calibration_uavs),
                                  ",".join(confirmed), len(self.all_uavs))
                else:
                    pending = [uav for uav in self.calibration_uavs
                               if not self.confirmed[uav]]
                    rospy.loginfo_throttle(5.0, "calibration ready pending UAVn: %s "
                                           "(confirmed=%d/%d)",
                                           ",".join(pending), len(confirmed),
                                           len(self.calibration_uavs))


def main():
    rospy.init_node("gcs_a_calibration")
    rospy.set_param("/group_a/calibration_ready", False)
    GcsACalibrationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
