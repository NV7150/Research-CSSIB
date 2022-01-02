import numpy
import numpy as np
from PointCloudCreator import create_point_cloud
import open3d as o3d


class TransformInput:
    def __init__(self, in_id=False):
        self.trans_vec = np.array([0, 0, 0], dtype=float)
        self.euler = np.array([0, 0, 0], dtype=float)
        self.f = np.array([0, 0], dtype=float)
        self.p = np.array([0, 0], dtype=float)
        self.is_exit = False
        self.in_id = in_id

    def transform_pinhole(self, ins):
        ins.fx += self.f[0]
        ins.fy += self.f[1]
        ins.ppx += self.p[0]
        ins.ppy += self.p[1]

        return ins

    def get_is_exit(self):
        return self.is_exit

    def get_r(self, base):
        return base.get_rotation_matrix_from_xyz(self.euler)

    def get_ext(self, base):
        r = o3d.geometry.get_rotation_matrix_from_xyz(self.euler)
        t = np.array(self.trans_vec).reshape([3, 1])
        rt = np.concatenate([r, t], axis=1)
        return np.concatenate([rt, np.array([[0, 0, 0, 1]])], axis=0)

    def input_s(self, s):
        offset = 0
        try:
            if s == 'exit':
                self.is_exit = True
                return

            if self.in_id:
                offset += 1
                if self.in_id != s[0]:
                    return

            n_i = 2 + offset
            o_i = 0 + offset
            d_i = 1 + offset
            n = float(s[n_i:])
            if s[o_i] == 't':
                if s[d_i] == 'x':
                    self.trans_vec[0] = n
                elif s[d_i] == 'y':
                    self.trans_vec[1] = n
                elif s[d_i] == 'z':
                    self.trans_vec[2] = n
            elif s[o_i] == 'r':
                if s[d_i] == 'x':
                    self.euler[0] = n
                elif s[d_i] == 'y':
                    self.euler[1] = n
                elif s[d_i] == 'z':
                    self.euler[2] = n
            elif s[o_i] == 'f':
                if s[d_i] == 'x':
                    self.f[0] = n
                elif s[d_i] == 'y':
                    self.f[1] = n
            elif s[o_i] == 'p':
                if s[d_i] == 'x':
                    self.p[0] = n
                elif s[d_i] == 'y':
                    self.p[1] = n

            print("trans:{}, rot:{}".format(self.trans_vec, self.euler))
        except ValueError or IndexError:
            print("Cannot process ", s)


def process_input(input_arr, trans_input, rs, source):
    for inp in input_arr:
        trans_input.input_s(inp)

    (rgb, depth, ins) = rs.get_frame_image()
    ins = trans_input.transform_pinhole(ins)

    if rgb is None or depth is None or ins is None:
        return

    # (rgb, depth) = bg_processor(rgb, depth)
    pcd = create_point_cloud(rgb, depth, ins, trans_input.get_ext(source))
    # pcd = trans_input.transform_pcd(pcd)

    return pcd

