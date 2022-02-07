import open3d as o3d
import numpy as np
import cv2


def create_point_cloud(color_image, depth_image, intrinsics, r, color=[255, 255, 255], depth_scale=1000):
    # depth_image = np.asarray(cv2.flip(depth_image, 1))
    # color_image = np.asarray(cv2.flip(color_image, 1))

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

    base_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    base_pcd = base_pcd.voxel_down_sample(voxel_size=0.01)

    pcd = base_pcd

    curr_pcd_points = np.asarray(base_pcd.points)

    def move(point):
        point = np.array([
            [point[0]],
            [point[1]],
            [point[2]],
            [1]
        ])
        return np.dot(r, point)[:3].flatten()

    for i in range(len(curr_pcd_points)):
        curr_pcd_points[i] = move(curr_pcd_points[i])

    moved_points = []
    pos = r[:3, 3].flatten()
    for p in curr_pcd_points:
        move_point = np.array([2 * pos[0] - p[0], p[1], 2 * pos[2] - p[2]])
        moved_points.append(move_point)

    pcd = o3d.geometry.PointCloud()

    # moved_points.extend(curr_pcd_points)
    colors = list(np.asarray(base_pcd.colors))
    # colors.extend(np.asarray(base_pcd.colors))

    pcd.points = o3d.utility.Vector3dVector(np.array(moved_points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

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
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    min_vals = np.array([100, 100, 100])
    for p in points:
        for (i, v) in enumerate(p):
            if min_vals[i] > v:
                min_vals[i] = v
    points -= min_vals

    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd, -min_vals


