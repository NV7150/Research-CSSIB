from BgProcessor import BgProcessor
from InputProcessor import InputProcessor, InputPool
from RealSense import RealSense
from RealSenseVis import RealSenseVis
from PointCloudCreator import create_point_cloud, load_pcd
from TransformInput import TransformInput


def simple_scan_vis():
    bg_processor = BgProcessor()
    queue = InputPool()
    input_s = InputProcessor("input", queue)
    input_s.start()
    trans_input = TransformInput()

    with RealSense() as real_sense:
        with RealSenseVis() as rs_vis:
            source = load_pcd("./sampleDatas/Room.ply")
            rs_vis.add_pcd(source)

            while not trans_input.get_is_exit():
                while queue.length() > 0:
                    trans_input.input_s(queue.shift())

                (rgb, depth, ins) = real_sense.get_frame_image()
                ins = trans_input.transform_pinhole(ins)

                if rgb is None or depth is None or ins is None:
                    continue

                (rgb, depth) = bg_processor(rgb, depth)
                pcd = create_point_cloud(rgb, depth, ins, trans_input.get_ext(source))
                pcd = trans_input.transform_pcd(pcd)

                rs_vis.visualize_current(pcd)


if __name__ == '__main__':
    simple_scan_vis()
