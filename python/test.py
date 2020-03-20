#!/usr/bin/env python3
import cv2
import numpy as np


def rotate_image(image, angle, scale):
    center = tuple(np.array(image.shape[0:2]) / 2)
    rot_mat = cv2.getRotationMatrix2D(center, angle, scale)
    return cv2.warpAffine(image,
                          rot_mat,
                          image.shape[0:2],
                          flags=cv2.INTER_CUBIC)


class CodeMapGui:
    max_zoom = 30
    camera_res = 64

    def __init__(self):
        # initial coordinates
        self.pos = [0, 0]
        self.rotation = 0
        self.zoom = 1
        # load code map
        self.code_map = cv2.imread('code_map.pbm', cv2.IMREAD_GRAYSCALE)
        # create code map window
        cv2.namedWindow('CodeMap',
                        flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                        | cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback('CodeMap', self.mouse_callback)
        cv2.moveWindow('CodeMap', 0, 0)
        cv2.resizeWindow('CodeMap', 640, 640)
        # create camera and control view
        cv2.namedWindow('Camera',
                        flags=cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        cv2.moveWindow('Camera', 970, 0)
        cv2.resizeWindow('Camera', 320, 320)
        cv2.namedWindow('ControlView',
                        flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                        | cv2.WINDOW_GUI_NORMAL)
        cv2.moveWindow('ControlView', 645, 0)
        cv2.resizeWindow('ControlView', 320, 320 + 50)
        cv2.createTrackbar('Rotation', 'ControlView', 180, 359,
                           self.rotation_callback)
        cv2.createTrackbar('Zoom', 'ControlView', CodeMapGui.max_zoom,
                           CodeMapGui.max_zoom * 2, self.zoom_callback)

    def run(self):
        cv2.imshow('CodeMap', self.code_map)
        self.frame_update()
        while (1):
            cv2.imshow('ControlView', self.control_view)
            cv2.imshow('Camera', self.camera)
            k = cv2.waitKey(50) & 0xFF
            if k == ord('m'):
                mode = not mode
            elif k == 27:
                break
        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        if flags == 2:
            self.pos[0] = np.clip(
                x - CodeMapGui.camera_res, 0,
                self.code_map.shape[0] - CodeMapGui.camera_res - 1)
            self.pos[1] = np.clip(
                y - CodeMapGui.camera_res, 0,
                self.code_map.shape[1] - CodeMapGui.camera_res - 1)
            self.frame_update()

    def rotation_callback(self, rotation):
        self.rotation = rotation - 180
        self.frame_update()

    def zoom_callback(self, zoom):
        self.zoom = 1 + (zoom - CodeMapGui.max_zoom) / 100
        self.frame_update()

    def frame_update(self):
        res = CodeMapGui.camera_res
        self.control_view = self.code_map[self.pos[1]:self.pos[1] +
                                          res * 2, self.pos[0]:self.pos[0] +
                                          res * 2]
        rot_mat = cv2.getRotationMatrix2D((0, 0), self.rotation,
                                          (res // 2) * self.zoom)
        corners = np.array([[-1, -1], [-1, 1], [1, 1], [1, -1]])
        for i, corner in enumerate(corners):
            corners[i] = np.int0(rot_mat[:, :2].dot(corner)) + res
        self.camera = rotate_image(self.control_view, self.rotation,
                                   self.zoom)[res // 2 - 1:res * 3 // 2 -
                                              1, res // 2 - 1:res * 3 // 2 - 1]
        self.control_view = cv2.cvtColor(self.control_view, cv2.COLOR_GRAY2RGB)
        cv2.drawContours(self.control_view, [corners], 0, (0, 255, 0), 2)


# create control window
code_map_gui = CodeMapGui()
code_map_gui.run()
