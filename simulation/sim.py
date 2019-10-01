#!/usr/bin/env python3
from PIL import Image, ImageFilter, ImageDraw
import ctypes

libsim = ctypes.CDLL("build/libsimulation.so")


class ImageMatrix(ctypes.Structure):
    _fields_ = [('data', ctypes.c_char_p), ('n_cols', ctypes.c_short),
                ('n_rows', ctypes.c_short)]

    def __init__(self, width, height, buf=None):
        self.buf = buf if buf else bytes(2 * width * height)
        self.data = self.buf
        self.n_cols = width
        self.n_rows = height

    def from_image(image):
        buf = image.tobytes() * 2 if image.mode == 'L' else image.convert(
            'L').tobytes() * 2
        image_matrix = ImageMatrix(image.width, image.height, buf)
        libsim.convert_uint8_to_int16(image_matrix)
        return image_matrix

    def to_image(self):
        libsim.normalize_elements(self, 255)
        libsim.convert_int16_to_uint8(self)
        image = Image.frombuffer('L', (self.n_cols, self.n_rows), self.buf,
                                 'raw', 'L', 0, 1)
        return image

    def print(self):
        libsim.print_matrix(self)

    def show(self, scale=10):
        self.to_image().resize(
            (int(self.n_cols * scale), int(self.n_rows * scale))).show()


libsim.threshold_elements.restype = None
libsim.threshold_elements.argtypes = [
    ImageMatrix, ctypes.c_short, ctypes.c_short
]

libsim.normalize_elements.restype = None
libsim.normalize_elements.argtypes = [ImageMatrix, ctypes.c_short]

libsim.edge_filter.restype = None
libsim.edge_filter.argtypes = [ctypes.POINTER(ImageMatrix), ImageMatrix]

libsim.hough_line_transform.restype = None
libsim.hough_line_transform.argtypes = [
    ctypes.POINTER(ImageMatrix), ImageMatrix
]

libsim.convert_int16_to_uint8.restype = None
libsim.convert_int16_to_uint8.argtypes = [ImageMatrix]

libsim.convert_uint8_to_int16.restype = None
libsim.convert_uint8_to_int16.argtypes = [ImageMatrix]

libsim.print_matrix.restype = None
libsim.print_matrix.argtypes = [ImageMatrix]

n = 30

code_map = Image.open("code_map.pbm").convert('L')
camera_image = code_map.rotate(30, Image.BILINEAR).crop(
    (500, 500, 500 + n, 500 + n))
camera_matrix = ImageMatrix.from_image(camera_image)

edge_matrix = ImageMatrix(n, n)
libsim.edge_filter(edge_matrix, camera_matrix)
#edge_matrix.print()

hough_matrix = ImageMatrix(600, 600)
libsim.hough_line_transform(hough_matrix, camera_matrix)
#hough_matrix.print()

edge_hough_matrix = ImageMatrix(600, 600)
libsim.normalize_elements(edge_matrix, 255)
#libsim.threshold_elements(edge_matrix, 200, 255)
libsim.hough_line_transform(edge_hough_matrix, edge_matrix)
#edge_hough_matrix.print()

#camera_matrix.show(10)
#edge_matrix.show(10)
edge_matrix.show(10)
#hough_matrix.show(1.1)
edge_hough_matrix.show(1.0)
