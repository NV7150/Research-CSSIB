import open3d as o3d
import numpy as np


def create_point_cloud(color_image, depth_image, intrinsics, r, color=[255, 255, 255], depth_scale=1000):
    img_depth = o3d.geometry.Image(depth_image)
    img_color = o3d.geometry.Image(color_image)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        img_color,
        img_depth,
        depth_scale=depth_scale,
        convert_rgb_to_intensity=False
    )

    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic, extrinsic=r)

    pos = [r[i, 3] for i in range(3)]
    for x in range(-5, 6):
        for y in range(-5, 6):
            for z in range(-5, 6):
                curr_p = np.array(pos, dtype=float)
                curr_p[0] += x * 0.005
                curr_p[1] += y * 0.005
                curr_p[2] += z * 0.005
                pcd.points.append(curr_p)
                pcd.colors.append(np.array(color, dtype=float))

    return pcd


def load_pcd(filename):
    return o3d.io.read_point_cloud(filename)