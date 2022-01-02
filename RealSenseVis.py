from RealSense import RealSense
import open3d as o3d
import numpy as np


class RealSenseVis(object):
    def __init__(self, key_dict=None):
        self.added_pc = {}
        self.key_dict = key_dict

    def __enter__(self):
        if self.key_dict is None:
            self.vis = o3d.visualization.Visualizer()
        else:
            print("activate keycallback")
            self.vis = o3d.visualization.VisualizerWithKeyCallback()
            for (key, action) in self.key_dict.items():
                print(key, action)
                self.vis.register_key_callback(key, action)

        self.vis.create_window('SCAN', width=1280, height=720)
        self.point_clouds = {}
        self.added_pc = {}

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.vis.destroy_window()
        del self.vis

    def visualize_current(self, pcd, rs_id=0):
        if not (rs_id in self.added_pc.keys()):
            self.added_pc.setdefault(rs_id, False)
            self.point_clouds.setdefault(rs_id, o3d.geometry.PointCloud())

        self.point_clouds[rs_id].points = pcd.points
        self.point_clouds[rs_id].colors = pcd.colors

        if not self.added_pc[rs_id] and len(np.asarray(pcd.points)) > 0:
            self.add_pcd(self.point_clouds[rs_id])
            self.added_pc[rs_id] = True

        self.vis.update_geometry(self.point_clouds[rs_id])
        self.vis.poll_events()
        self.vis.update_renderer()

    def add_pcd(self, pcd):
        self.vis.add_geometry(pcd)
