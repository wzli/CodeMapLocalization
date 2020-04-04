#!/usr/bin/env python3

import cv2
import codemap.core


class WebCamLocalization:
    def __init__(self, device_num):
        self.cam = cv2.VideoCapture(device_num)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 64)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 64)
        self.loc_ctx = codemap.core.LocalizationContext()
        self.latest_message = {}

    def update(self):
        # read frame
        success, frame = self.cam.read()
        if not success:
            return None
        # convert to grayscale
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # crop center square
        center = frame.shape[1] // 2
        offset = frame.shape[0] // 2
        frame = frame[:, center - offset:center + offset]
        # resize to 64x64
        frame = cv2.resize(frame, (64, 64))
        # run localization
        self.loc_ctx.run(frame)
        self.latest_message = self.loc_ctx.get_location_msg()
        return self.latest_message

    def run(self, callback):
        while True:
            if self.update():
                callback(self.latest_message)
