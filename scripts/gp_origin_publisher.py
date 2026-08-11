#!/usr/bin/env python3
"""GCS_A 全局原点发布器。

按 1Hz 周期向 /group_a/gp_origin 发布 GeoPointStamped 原点（经纬高）。
所有无人机在起飞前经 bridge 收到后持续设置本机
/mavros/global_position/set_gp_origin，使各机 local ENU 坐标系对齐到同一原点。
相比 safe_valley_exp 依赖 leader 位置统一下发，GCS_A 统一下发更稳定，且无人机
起飞前持续设置可规避 RTK 定位完成较晚导致的坐标突变。

参数:
  ~origin_lat  默认 28.2（湖南大学科技园附近）
  ~origin_lon  默认 112.9
  ~origin_alt  默认 50.0
"""
import rospy
from geographic_msgs.msg import GeoPointStamped


class GpOriginPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/group_a/gp_origin", GeoPointStamped, queue_size=1)
        self.origin_lat = rospy.get_param("~origin_lat", 28.2)
        self.origin_lon = rospy.get_param("~origin_lon", 112.9)
        self.origin_alt = rospy.get_param("~origin_alt", 50.0)
        rospy.loginfo("gp_origin_publisher: origin lat=%.6f lon=%.6f alt=%.1f",
                      self.origin_lat, self.origin_lon, self.origin_alt)

    def publish_origin_loop(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            msg = GeoPointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.position.latitude = self.origin_lat
            msg.position.longitude = self.origin_lon
            msg.position.altitude = self.origin_alt
            self.pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("gp_origin_publisher")
    GpOriginPublisher().publish_origin_loop()


if __name__ == "__main__":
    main()