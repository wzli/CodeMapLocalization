#!/usr/bin/env python3
import cv2
import numpy as np
import code_map as cm


def rotate_image(image, angle, scale):
    center = tuple(np.array(image.shape[0:2]) / 2)
    rot_mat = cv2.getRotationMatrix2D(center, angle, scale)
    return cv2.warpAffine(image,
                          rot_mat,
                          image.shape[0:2],
                          flags=cv2.INTER_CUBIC)


def create_window(name, size, pos=None):
    cv2.namedWindow(name,
                    flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                    | cv2.WINDOW_GUI_NORMAL)
    if pos is not None:
        cv2.moveWindow(name, *pos)
    cv2.resizeWindow(name, *size)


class CodeMapGui:
    CAMERA_RES = 64
    MAX_ZOOM = 20

    def __init__(self):
        # initial coordinates
        self.pos = np.array([0, 0])
        self.rotation = 0
        self.zoom = 1
        self.blur = 0
        self.tunnel = np.ones((CodeMapGui.CAMERA_RES, CodeMapGui.CAMERA_RES))
        self.noise = 0
        # initialize localization context
        self.loc_ctx = cm.LocalizationContext()
        # load code map
        self.code_map = cv2.imread('code_map.pbm', cv2.IMREAD_GRAYSCALE)
        # create code map window
        create_window('CodeMap', (512, 512))
        cv2.setMouseCallback('CodeMap', self.code_map_mouse_callback)
        # create navigate window
        create_window('Navigate', (512, 512 + 50))
        cv2.createTrackbar('Rotation', 'Navigate', 0, 360,
                           self.rotation_callback)
        cv2.createTrackbar('Zoom', 'Navigate', CodeMapGui.MAX_ZOOM,
                           CodeMapGui.MAX_ZOOM * 2, self.zoom_callback)
        cv2.setMouseCallback('Navigate', self.navigate_mouse_callback)
        # create Pipeline window
        create_window('Pipeline', (3 * 256, 2 * 256 + 75))
        cv2.createTrackbar('Blur', 'Pipeline', 0, 30, self.blur_callback)
        cv2.createTrackbar('Tunnel', 'Pipeline', 0, 30, self.tunnel_callback)
        cv2.createTrackbar('Noise', 'Pipeline', 0, 30, self.noise_callback)

    def update_frame(self):
        res = CodeMapGui.CAMERA_RES
        # clip position to be within boundary
        self.pos[0] = np.clip(self.pos[0], 0,
                              self.code_map.shape[0] - (res // 2) - 1)
        self.pos[1] = np.clip(self.pos[1], 0,
                              self.code_map.shape[1] - (res // 2) - 1)
        # sync rotation track bar
        cv2.setTrackbarPos('Rotation', 'Navigate', (self.rotation) % 360)
        # crop navigate view from code map
        self.navigate = self.code_map[self.pos[1]:self.pos[1] +
                                      res, self.pos[0]:self.pos[0] + res]
        # rotate and zoom from navigate view to get camera view
        rot_mat = cv2.getRotationMatrix2D((0, 0), -self.rotation,
                                          2 + res / (6 * self.zoom))
        self.camera = rotate_image(self.navigate, self.rotation, 3 * self.zoom)
        # draw box around camera view
        self.navigate = cv2.cvtColor(self.navigate, cv2.COLOR_GRAY2RGB)
        corners = np.array([[-1, -1], [-1, 1], [1, 1], [1, -1]])
        for i, corner in enumerate(corners):
            corners[i] = np.int0(rot_mat[:, :2].dot(corner)) + (res // 2)
        cv2.drawContours(self.navigate, [corners], 0, (0, 255, 0), 2)
        # convert to float to apply camera distortions
        self.camera = np.float32(self.camera)
        # apply blur
        self.camera = cv2.GaussianBlur(self.camera,
                                       (2 * (self.blur // 10) + 1, 2 *
                                        (self.blur // 10) + 1), self.blur / 20)
        # apply tunnel
        self.camera *= self.tunnel
        # apply noise
        noise = np.empty(self.camera.shape, dtype=np.float32)
        cv2.randn(noise, 0, self.noise)
        self.camera += noise
        self.camera = cv2.convertScaleAbs(self.camera)
        # run localization algorithm
        self.loc_ctx.run(self.camera)
        # create pipeline image
        self.pipeline = np.zeros((res * 2, res * 3), dtype=np.ubyte) + 127
        # top left
        self.pipeline[:res, :res] = self.camera
        # top mid
        self.pipeline[:res, res:(2 * res)] = self.loc_ctx.derotated_image_array
        # top right
        self.pipeline[:res - 2, (2 * res):(3 * res) -
                      2] = self.loc_ctx.sharpened_image_array
        # bottom right
        self.pipeline[res:(2 * res), (2 * res):(
            3 * res)] = self.loc_ctx.thresholded_image_array
        # bottom mid
        self.pipeline[res:(2 * res), res:(2 * res)] = cv2.resize(
            self.loc_ctx.extracted_image_array, (3 * res // 2, 3 * res // 2),
            interpolation=cv2.INTER_NEAREST)[:res, :res]
        # bottom left
        self.pipeline[res:(2 * res), :res] = cv2.resize(
            self.loc_ctx.centered_correlation, (res, res))
        # debug prints
        radians = np.radians(self.rotation)
        if radians > np.pi:
            radians -= 2 * np.pi
        print(
            f'\nGround Truth\t{{\"x\":{self.pos[0] + 24}, \"y\":{self.pos[1] + 24}, \"rot\":{round(radians, 6)}}}'
        )
        self.loc_ctx.print()

    def run(self):
        self.update_frame()
        cv2.imshow('CodeMap', self.code_map)
        while (self.key_callback(cv2.waitKey(50))):
            cv2.imshow('Navigate', self.navigate)
            cv2.imshow('Pipeline', self.pipeline)
        cv2.destroyAllWindows()

    def key_callback(self, key):
        if key == -1:
            return True
        key = key & 0xFF
        if key == 27:  # escape key
            return False
        elif key == ord('w'):
            self.pos[1] -= 1
        elif key == ord('a'):
            self.pos[0] -= 1
        elif key == ord('s'):
            self.pos[1] += 1
        elif key == ord('d'):
            self.pos[0] += 1
        elif key == ord('q'):
            self.rotation -= 2
        elif key == ord('e'):
            self.rotation += 2
        elif key == ord('r'):
            self.increment_zoom(1)
        elif key == ord('f'):
            self.increment_zoom(-1)
        self.update_frame()
        return True

    def navigate_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN:
            self.prev_mouse_pos = np.array((x, y))
        elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
            del self.prev_mouse_pos
        elif event == cv2.EVENT_MOUSEMOVE and hasattr(self, 'prev_mouse_pos'):
            mouse_pos = np.array((x, y))
            displacement = mouse_pos - self.prev_mouse_pos
            self.prev_mouse_pos = mouse_pos
            if flags == cv2.EVENT_FLAG_LBUTTON:
                self.pos -= displacement
            elif flags == cv2.EVENT_FLAG_RBUTTON:
                self.rotation += 2 * displacement[0]
                self.increment_zoom(-displacement[1])
            self.update_frame()

    def code_map_mouse_callback(self, event, x, y, flags, param):
        if flags == cv2.EVENT_FLAG_RBUTTON:
            border = CodeMapGui.CAMERA_RES // 2
            self.pos[0] = x - border
            self.pos[1] = y - border
            self.update_frame()

    def rotation_callback(self, rotation):
        self.rotation = rotation
        self.update_frame()

    def zoom_callback(self, val):
        self.zoom = 1 + (val - CodeMapGui.MAX_ZOOM) / 100
        self.update_frame()

    def blur_callback(self, val):
        self.blur = val
        self.update_frame()

    def tunnel_callback(self, val):
        self.tunnel = val
        envelope = np.arange(-1,
                             1.01,
                             2 / (CodeMapGui.CAMERA_RES - 1),
                             dtype=np.float32)
        envelope = np.exp(-(envelope**2) * val / 20)
        self.tunnel = np.outer(envelope, envelope)
        self.update_frame()

    def noise_callback(self, val):
        self.noise = val
        self.update_frame()

    def increment_zoom(self, inc):
        zoom_track = cv2.getTrackbarPos('Zoom', 'Navigate')
        zoom_track = np.clip(zoom_track + inc, 0, 2 * CodeMapGui.MAX_ZOOM)
        cv2.setTrackbarPos('Zoom', 'Navigate', zoom_track)


# create control window
code_map_gui = CodeMapGui()
code_map_gui.run()
