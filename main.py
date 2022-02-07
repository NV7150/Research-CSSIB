import datetime
import glob

from AccelSensor import AccelSensor
from BgProcessor import BgProcessor
from InputProcessor import InputProcessor, InputPool
from KeyInputter import *
from RealSense import RealSense
from RealSenseVis import RealSenseVis
from PointCloudCreator import create_point_cloud, load_pcd
from TransformInput import TransformInput, process_input
import open3d as o3d
from QrCodeDefiner import *

def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def simple_scan_vis():
    inputter = KeyInputter(1, 2)
    key_dict = register_keys(inputter, 0.01)

    with RealSense(serial_num='135322064678') as real_sense1:
        with RealSense(serial_num='142422061877') as real_sense2:
            with RealSenseVis(key_dict=key_dict) as rs_vis:
                source = load_pcd('sampleDatas/Room2.ply')
                rs_vis.add_pcd(source)

                while not inputter.is_exit:
                    pcd1 = process_key_input(0, inputter, real_sense1, color=[255, 0, 0])
                    if not (pcd1 is None):
                        rs_vis.visualize_current(pcd1, rs_id=0)

                    pcd2 = process_key_input(1, inputter, real_sense2, color=[0, 255, 0])
                    if not (pcd2 is None):
                        rs_vis.visualize_current(pcd2, rs_id=1)


def auto_regis_vis(auto_regis=True):
    inputter = KeyInputter(1, 1)
    key_dict = register_keys(inputter, 0.01)
    source, translation = load_pcd('sampleDatas/RoomScan.ply')

    if auto_regis:
        pos_dict = define_qr_pos(glob.glob("sampleDatas/RoomScan/frame*.json"), source)
        pos = np.array(pos_dict['135322064678']) + translation
        inputter.apply_pos(pos, 0)

    with RealSense(serial_num='135322064678') as real_sense:
        with RealSenseVis(key_dict=key_dict) as rs_vis:
            with AccelSensor("COM3", 9600) as ac_sensor:
                rs_vis.add_pcd(source)
                while not inputter.is_exit:
                    pcd = process_key_input(0, inputter, real_sense, color=[255, 0, 0])

                    if inputter.get_is_save():
                        o3d.io.write_point_cloud(f"{str(datetime.datetime.now()).split()[-1].split('.')[-1]}.ply", pcd)

                    if auto_regis:
                        angle = ac_sensor.get_xy_angles()
                        angle = preprocess_angle(angle)
                        inputter.apply_angle(angle, 0)

                    if not (pcd is None):
                        rs_vis.visualize_current(pcd, rs_id=0)


if __name__ == '__main__':
    auto_regis_vis()
