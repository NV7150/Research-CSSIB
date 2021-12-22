from RealSense import RealSense
import open3d as o3d
import numpy as np


class RealSenseVis(object):
    def __init__(self):
        self.added_pc = False

    def __enter__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window('SCAN', width=1280, height=720)
        self.point_cloud = o3d.geometry.PointCloud()
        self.added_pc = False

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.vis.destroy_window()
        del self.vis

    def visualize_current(self, pcd):
        self.point_cloud.points = pcd.points
        self.point_cloud.colors = pcd.colors

        if not self.added_pc and len(np.asarray(pcd.points)) > 0:
            self.add_pcd(self.point_cloud)
            self.added_pc = True

        self.vis.update_geometry(self.point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def add_pcd(self, pcd):
        self.vis.add_geometry(pcd)
