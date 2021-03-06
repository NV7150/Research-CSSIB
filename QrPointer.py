import numpy as np
import math
import open3d as o3d

from FrameLoader import Frame
from QrCodeDecoder import QRCode


# frame中のQRコードを発見し，QRコード方向にzだけいった地点の三次元座標を返す
def get_qr_from_frame(qrcode: QRCode, frame: Frame, z=1, trans=None):
    if trans is None:
        trans = np.zeros(shape=3)

    center = qrcode.get_point()
    inv_int = np.linalg.inv(frame.intrinsics)
    image_pos = np.array([[center[0]], [center[1]], [1]]) * z

    position_camera_pos = np.dot(inv_int, image_pos)
    position_camera_pos = np.concatenate([position_camera_pos, np.array([[1]])])

    position_global_pos = np.dot(frame.pose, position_camera_pos)
    position_global_pos = position_global_pos.flatten()[:3]

    return -(position_global_pos - frame.pos) + frame.pos + trans


def cal_dist(p1, p2):
    return math.sqrt(np.sum(np.power(p1 - p2, 2)))


def get_min_dist(pos, points):
    min_dist = cal_dist(points[0], pos)
    min_i = 0
    dist_list = []

    for (i, point) in enumerate(points[1:]):
        d = cal_dist(point, pos)
        dist_list.append(d)
        if min_dist > d:
            min_dist = d
            min_i = i

    return min_dist, min_i, dist_list


def ray_cast_qr(frame: Frame, point_dist, pcd, step=-0.025, th=0.01):
    def line_func(t):
        dir_vec = (frame.pos - point_dist)
        vec_l = np.linalg.norm(dir_vec)
        direction = dir_vec / vec_l
        return frame.pos + direction * t

    curr_t = 0
    curr_points = np.array(pcd.points)[:]
    res_d = math.inf
    res_point = 0
    res_true = None

    while True:
        curr_t += step
        line_point = line_func(curr_t)
        (min_dist, min_i, dist_list) = get_min_dist(line_point, curr_points)

        if res_d < min_dist and abs(min_dist - res_d) > th:
            break

        res_point = curr_points[min_i]
        res_d = min_dist
        res_true = line_point

        delete_th = min_dist * 2
        delete_list = [i for (i, dist) in enumerate(dist_list) if dist > delete_th]
        curr_points = np.delete(curr_points, delete_list, axis=0)

    return res_point, res_d, res_true


def ray_cast_qr_mesh(frame: Frame, destination, mesh_scene, trans=None):
    if trans is None:
        trans = np.arange([0, 0, 0])

    origin_pos = frame.pos + trans

    vec_to_dest = (origin_pos - destination)
    vec_l = np.linalg.norm(vec_to_dest)
    direction = vec_to_dest / vec_l

    ray_vec = np.concatenate([origin_pos, -direction])
    ray = o3d.core.Tensor([ray_vec], dtype=o3d.core.Dtype.Float32)
    ans = mesh_scene.cast_rays(ray)

    hit_dist = ans['t_hit'].numpy()[0]
    hit_pos = -direction * hit_dist + origin_pos

    return hit_pos

