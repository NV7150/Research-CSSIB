from BgProcessor import BgProcessor
from InputProcessor import InputProcessor, InputPool
from KeyInputter import KeyInputter, register_keys, process_key_input
from RealSense import RealSense
from RealSenseVis import RealSenseVis
from PointCloudCreator import create_point_cloud, load_pcd
from TransformInput import TransformInput, process_input


def simple_scan_vis():
    inputter = KeyInputter(1, 2)
    key_dict = register_keys(inputter, 0.01)

    with RealSense(serial_num='135322064678') as real_sense1:
        with RealSense(serial_num='142422061877') as real_sense2:
            with RealSenseVis(key_dict=key_dict) as rs_vis:
                source = load_pcd('sampleDatas/Room2.ply')
                rs_vis.add_pcd(source)

                while not inputter.is_exit:
                    pcd1 = process_key_input(0, inputter, real_sense1)
                    if not (pcd1 is None):
                        rs_vis.visualize_current(pcd1, rs_id=0)

                    pcd2 = process_key_input(1, inputter, real_sense2)
                    if not (pcd2 is None):
                        rs_vis.visualize_current(pcd2, rs_id=1)


if __name__ == '__main__':
    simple_scan_vis()
