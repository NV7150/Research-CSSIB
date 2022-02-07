import math

import keyboard
import numpy as np
import threading
from enum import Enum
import open3d as o3d

from PointCloudCreator import create_point_cloud
from RealSense import RealSense


class Position(Enum):
    X = 0
    Y = 1
    Z = 2


class Transform(Enum):
    TRANSLATE = 0
    ROTATION = 1


class LockHandler:
    def __init__(self, lock):
        self.lock = lock

    def __enter__(self):
        self.lock.acquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release()


class KeyInputter:
    def __init__(self, step_rot, num):
        self.transform = np.array([[
            [0, 0, 0],
            [0, 0, 0]
        ] for i in range(num)], dtype=float)
        print(self.transform)
        self.transform_sup = np.array([[
            [0, 0, 0],
            [0, 0, 0]
        ] for i in range(num)], dtype=float)
        self.lock = threading.Lock()
        self.pos_mode = Position.X
        self.trans_mode = Transform.TRANSLATE
        self.step_rot = step_rot
        self.target = 0
        self.n = num
        self.is_exit = False
        self.is_save = False

    def change_mode(self, mode: Transform):
        with LockHandler(self.lock):
            self.trans_mode = mode
            print(f"change mode: {mode}")

    def change_pos(self, pos_mode: Position):
        with LockHandler(self.lock):
            self.pos_mode = pos_mode
            print(f"change pos parameter: {pos_mode}")

    def change_target(self, target):
        if target > self.n or target < 0:
            return

        with LockHandler(self.lock):
            self.target = target
            print(f"change target: {target}")

    def transform_with(self, val):
        with LockHandler(self.lock):
            if self.trans_mode == Transform.ROTATION:
                val *= self.step_rot
            self.transform[int(self.target), int(self.trans_mode.value), int(self.pos_mode.value)] += float(val)

            print(f"target: {self.target} transformed to {self.transform[self.target]}")

    def switch_target(self):
        self.target += 1
        self.target = self.target if self.target < self.n else 0

    def exit(self):
        self.is_exit = True

    def export_matrix(self, target):
        with LockHandler(self.lock):
            trans = self.transform + self.transform_sup
            r = o3d.geometry.get_rotation_matrix_from_xyz(trans[target, Transform.ROTATION.value])
            t = np.array(trans[target, Transform.TRANSLATE.value]).reshape([3, 1])
            rt = np.concatenate([r, t], axis=1)
            export_mat = np.concatenate([rt, np.array([[0, 0, 0, 1]])], axis=0, dtype=float)
        return export_mat

    def input_matrix(self):
        s = input("input format:(target), (t_x), (t_y), (t_z), (r_x), (r_y), (r_z)")
        try:
            vals = list(map(float, s.split(",")))
        except ValueError:
            print("invalid input, aborting")
            return

        if len(vals) != 7:
            print("invalid input, aborting")
            return
        val_arr = np.array([[vals[i] for i in range(1, 4)], [vals[i] for i in range(4, 7)]])
        self.transform[int(vals[0])] = val_arr

    def apply_angle(self, angle_mat, target):
        prev = self.transform_sup[target, Transform.ROTATION.value]
        self.transform_sup[target, Transform.ROTATION.value] = [angle_mat[0], prev[1], angle_mat[1]]

    def apply_pos(self, pos, target):
        self.transform_sup[target, Transform.TRANSLATE.value] = pos

    def get_is_save(self):
        if self.is_save:
            self.is_save = False
            return True
        return False

    def set_is_save(self):
        self.is_save = True


def register_keys(inputter: KeyInputter, step):
    print(
        "t: translate mode, r: rotation mode \n" +
        "x: x_axis, y: y_axis, z: z_axis \n" +
        "s: switch target(e.g. 0->1, 1->0)\n" +
        "i: input manually\n" +
        "1/0: increase/decrease little, 2/9: increase/decrease, 3/8: increase/decrease big\n" +
        "e: exit"
    )

    def lambda_create(func):
        def f(vis):
            func()
            return False

        return f
    key_to_callback = {
        ord("T"): lambda_create(lambda: inputter.change_mode(Transform.TRANSLATE)),
        ord("R"): lambda_create(lambda: inputter.change_mode(Transform.ROTATION)),
        ord("X"): lambda_create(lambda: inputter.change_pos(Position.X)),
        ord("Y"): lambda_create(lambda: inputter.change_pos(Position.Y)),
        ord("Z"): lambda_create(lambda: inputter.change_pos(Position.Z)),
        ord("Q"): lambda_create(lambda: inputter.exit()),
        ord("S"): lambda_create(lambda: inputter.switch_target()),
        ord("I"): lambda_create(lambda: inputter.input_matrix()),
        ord("S"): lambda_create(lambda: inputter.set_is_save()),
        ord("1"): lambda_create(lambda: inputter.transform_with(step)),
        ord("2"): lambda_create(lambda: inputter.transform_with(step * 3)),
        ord("3"): lambda_create(lambda: inputter.transform_with(step * 10)),
        ord("0"): lambda_create(lambda: inputter.transform_with(-step)),
        ord("9"): lambda_create(lambda: inputter.transform_with(-step * 3)),
        ord("8"): lambda_create(lambda: inputter.transform_with(-step * 10))
    }

    return key_to_callback

    # keyboard.on_press_key("t", lambda _: inputter.change_mode(Transform.TRANSLATE))
    # keyboard.on_press_key("o", lambda _: inputter.change_mode(Transform.ROTATION))
    # keyboard.on_press_key("x", lambda _: inputter.change_pos(Position.X))
    # keyboard.on_press_key("y", lambda _: inputter.change_pos(Position.Y))
    # keyboard.on_press_key("z", lambda _: inputter.change_pos(Position.Z))
    # keyboard.on_press_key("p", lambda _: inputter.change_target(0))
    # keyboard.on_press_key("o", lambda _: inputter.change_target(1))
    # keyboard.on_press_key("i", lambda _: inputter.switch_target())
    # keyboard.on_press_key("e", lambda _: inputter.exit())
    # keyboard.on_press_key("a", lambda _: inputter.transform_with(step))
    # keyboard.on_press_key("s", lambda _: inputter.transform_with(step * 3))
    # keyboard.on_press_key("d", lambda _: inputter.transform_with(step * 10))
    # keyboard.on_press_key(";", lambda _: inputter.transform_with(-step))
    # keyboard.on_press_key("l", lambda _: inputter.transform_with(-step * 3))
    # keyboard.on_press_key("k", lambda _: inputter.transform_with(-step * 10))


def process_key_input(target, key_input: KeyInputter, rs: RealSense, color=None):
    if color is None:
        color = [255, 255, 255]
    (rgb, depth, ins) = rs.get_frame_image()
    pcd = create_point_cloud(rgb, depth, ins, key_input.export_matrix(target), color=color)

    return pcd


def preprocess_angle(raw_angle):
    angle = [raw_angle[1], 0, raw_angle[0]]
    angle[0] += math.pi

    return -np.array(angle)


def preprocess_pos(raw_pos):
    return np.array([-raw_pos[0], raw_pos[1], raw_pos[2]])
