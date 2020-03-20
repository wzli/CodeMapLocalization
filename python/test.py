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


def create_window(name, size, pos):
    cv2.namedWindow(name,
                    flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                    | cv2.WINDOW_GUI_NORMAL)
    cv2.moveWindow(name, *pos)
    cv2.resizeWindow(name, *size)


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
        create_window('CodeMap', (640, 640), (0, 0))
        cv2.setMouseCallback('CodeMap', self.mouse_callback)
        # create camera and control view
        create_window('Camera', (320, 320), (970, 0))
        create_window('ControlView', (320, 320 + 50), (645, 0))
        cv2.createTrackbar('Rotation', 'ControlView', 180, 360,
                           self.rotation_callback)
        cv2.createTrackbar('Zoom', 'ControlView', CodeMapGui.max_zoom,
                           CodeMapGui.max_zoom * 2, self.zoom_callback)

    def run(self):
        cv2.imshow('CodeMap', self.code_map)
        self.frame_update()
        while (self.key_callback(cv2.waitKey(50))):
            cv2.imshow('ControlView', self.control_view)
            cv2.imshow('Camera', self.camera)
        cv2.destroyAllWindows()

    def key_callback(self, key):
        key = key & 0xFF
        if key == ord('m'):
            mode = not mode
        elif key == 27:
            return False
        return True

    def mouse_callback(self, event, x, y, flags, param):
        if flags == 2:
            border = CodeMapGui.camera_res // 2
            self.pos[0] = np.clip(x - border, 0,
                                  self.code_map.shape[0] - border - 1)
            self.pos[1] = np.clip(y - border, 0,
                                  self.code_map.shape[1] - border - 1)
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
                                          res, self.pos[0]:self.pos[0] + res]
        rot_mat = cv2.getRotationMatrix2D((0, 0), self.rotation,
                                          2 + res / (6 * self.zoom))
        self.camera = rotate_image(self.control_view, self.rotation,
                                   3 * self.zoom)

        # draw box around camera view
        self.control_view = cv2.cvtColor(self.control_view, cv2.COLOR_GRAY2RGB)
        corners = np.array([[-1, -1], [-1, 1], [1, 1], [1, -1]])
        for i, corner in enumerate(corners):
            corners[i] = np.int0(rot_mat[:, :2].dot(corner)) + (res // 2)
        cv2.drawContours(self.control_view, [corners], 0, (0, 255, 0), 2)


# create control window
code_map_gui = CodeMapGui()
code_map_gui.run()
