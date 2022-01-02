import pyrealsense2 as rs
import numpy as np


class RealSense(object):
    def __init__(self, serial_num=False):
        if serial_num:
            self.use_serial = True
            self.serial_num = serial_num
        else:
            self.use_serial = False

    def __enter__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if self.use_serial:
            self.config.enable_device(self.serial_num)

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pipeline.stop()

    def get_frame_image(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        profile = frames.get_profile()

        if not depth_frame or not color_frame:
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image, profile.as_video_stream_profile().get_intrinsics()
