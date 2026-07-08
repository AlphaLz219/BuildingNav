#!/usr/bin/env python3
"""3D 点云 → 2D 占据栅格投影。

从 Fast-LIO 的 /cloud_registered 累积点云，按固定 Z 高度范围投影到 2D 栅格，
发布 /map (nav_msgs/OccupancyGrid)。栅格只增长不清空，形成持续扩展的地图。
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import sensor_msgs.point_cloud2 as pc2

# ======================== 参数 ========================
MAP_RESOLUTION   = 0.05
MAP_ORIGIN_X     = -20.0
MAP_ORIGIN_Y     = -10.0
MAP_WIDTH_CELLS  = 1200
MAP_HEIGHT_CELLS = 1600
PUBLISH_RATE     = 2.0
MAX_POINTS       = 500000
# Z 切片范围 (camera_init 系): 地板约在 z=-0.16 ~ z=0.2 高度
Z_MIN            = -0.5
Z_MAX            = 0.8


class PointCloudTo2DMap:
    def __init__(self):
        self._grid = np.full((MAP_HEIGHT_CELLS, MAP_WIDTH_CELLS), -1, dtype=np.int8)
        self._accumulated_pts = []
        self._robot_x = 0.0
        self._robot_y = 0.0

        self._pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=5, latch=True)
        self._sub_cloud = rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_cb)
        self._sub_odom = rospy.Subscriber("/Odometry", Odometry, self._odom_cb)
        rospy.loginfo("3D→2D Map projector ready   Z range: [%.1f, %.1f]", Z_MIN, Z_MAX)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._robot_x = p.x
        self._robot_y = p.y

    def _cloud_cb(self, msg: PointCloud2):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return
        # 下采样以控制内存
        step = max(1, len(points) // 3000)
        for pt in points[::step]:
            self._accumulated_pts.append((pt[0], pt[1], pt[2]))
        if len(self._accumulated_pts) > MAX_POINTS:
            self._accumulated_pts = self._accumulated_pts[-MAX_POINTS:]

    def _xy_to_index(self, x, y):
        ix = int((x - MAP_ORIGIN_X) / MAP_RESOLUTION)
        iy = int((y - MAP_ORIGIN_Y) / MAP_RESOLUTION)
        if 0 <= ix < MAP_WIDTH_CELLS and 0 <= iy < MAP_HEIGHT_CELLS:
            return ix, iy
        return None

    def _update_map(self):
        """增量更新：不清空旧格子，只追加新点 → 占用。"""
        n_occupied = 0
        for x, y, z in self._accumulated_pts:
            if z < Z_MIN or z > Z_MAX:
                continue
            idx = self._xy_to_index(x, y)
            if idx is None:
                continue
            if self._grid[idx[1], idx[0]] < 50:
                self._grid[idx[1], idx[0]] = 100
                n_occupied += 1

        # 机器人周围标记空闲
        robot_idx = self._xy_to_index(self._robot_x, self._robot_y)
        if robot_idx is not None:
            r = int(0.6 / MAP_RESOLUTION)  # 0.6m 半径
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if di*di + dj*dj > r*r:
                        continue
                    ni, nj = robot_idx[0] + di, robot_idx[1] + dj
                    if 0 <= ni < MAP_WIDTH_CELLS and 0 <= nj < MAP_HEIGHT_CELLS:
                        if self._grid[nj, ni] <= 0:
                            self._grid[nj, ni] = 0

        return n_occupied

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_init"
        msg.info = MapMetaData(
            resolution=MAP_RESOLUTION,
            width=MAP_WIDTH_CELLS,
            height=MAP_HEIGHT_CELLS,
            origin=msg.info.origin
        )
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = MAP_WIDTH_CELLS
        msg.info.height = MAP_HEIGHT_CELLS
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self._grid.flatten().tolist()
        self._pub_map.publish(msg)

    def run(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            n = self._update_map()
            self._publish_map()
            if n > 0:
                rospy.loginfo_throttle(5,
                    f"Map: {MAP_WIDTH_CELLS}×{MAP_HEIGHT_CELLS}"
                    f" | pts_buf: {len(self._accumulated_pts)}"
                    f" | new_occupied: {n}")
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_2d_map")
    PointCloudTo2DMap().run()
