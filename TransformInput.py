import numpy
import numpy as np


class TransformInput:
    def __init__(self):
        self.trans_vec = np.array([0, 0, 0], dtype=float)
        self.euler = np.array([0, 0, 0], dtype=float)
        self.f = np.array([0, 0], dtype=float)
        self.p = np.array([0, 0], dtype=float)
        self.is_exit = False

    def transform_pcd(self, pcd):
        # pcd.translate(self.trans_vec)
        # r = base.get_rotation_matrix_from_xyz(self.euler)
        # pcd.rotate(r)
        return pcd

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
        r = base.get_rotation_matrix_from_xyz(self.euler)
        t = np.array(self.trans_vec).reshape([3, 1])
        rt = np.concatenate([r, t], axis=1)
        return np.concatenate([rt, np.array([[0, 0, 0, 1]])], axis=0)


    def input_s(self, s):
        try:
            if s == 'exit':
                self.is_exit = True

            n = float(s[2:])
            if s[0] == 't':
                if s[1] == 'x':
                    self.trans_vec[0] = n
                elif s[1] == 'y':
                    self.trans_vec[1] = n
                elif s[1] == 'z':
                    self.trans_vec[2] = n
            elif s[0] == 'r':
                if s[1] == 'x':
                    self.euler[0] = n
                elif s[1] == 'y':
                    self.euler[1] = n
                elif s[1] == 'z':
                    self.euler[2] = n
            elif s[0] == 'f':
                if s[1] == 'x':
                    self.f[0] = n
                elif s[1] == 'y':
                    self.f[1] = n
            elif s[0] == 'p':
                if s[1] == 'x':
                    self.p[0] = n
                elif s[1] == 'y':
                    self.p[1] = n


            print("trans:{}, rot:{}".format(self.trans_vec, self.euler))
        except ValueError:
            print("Cannot process ", s)

