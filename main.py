from BgProcessor import BgProcessor
from RealSense import RealSense
from RealSenseVis import RealSenseVis
from PointCloudCreator import create_point_cloud, load_pcd


def simple_scan_vis():
    bg_processor = BgProcessor()

    with RealSense() as real_sense:
        with RealSenseVis() as rs_vis:
            rs_vis.add_pcd(load_pcd("./sampleDatas/Room.ply"))

            while True:
                (rgb, depth, ins) = real_sense.get_frame_image()

                if rgb is None or depth is None or ins is None:
                    continue

                (rgb, depth) = bg_processor(rgb, depth)
                pcd = create_point_cloud(rgb, depth, ins)

                rs_vis.visualize_current(pcd)


if __name__ == '__main__':
    simple_scan_vis()