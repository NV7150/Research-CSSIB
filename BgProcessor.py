import cv2
import numpy as np


class BgProcessor:
    def __init__(self):
        self.model = cv2.bgsegm.createBackgroundSubtractorGSOC()

    def __call__(self, process_image, process_depth):
        mask = self.model.apply(process_image)

        process_image[mask == 0] = np.asarray([0, 0, 0])
        process_depth[mask == 0] = 0

        return process_image, process_depth
