#!/usr/bin/env python3
from PIL import Image, ImageFilter, ImageDraw
import ctypes

libsim = ctypes.CDLL("build/libsimulation.so")

class ImageMatrix(ctypes.Structure):
    _fields_ = [
        ('data', ctypes.c_char_p),
        ('width', ctypes.c_ushort),
        ('height', ctypes.c_ushort)
    ]

    def __init__(self, width, height, buf = None) :
        self.buf = buf if buf else bytes(2 * width * height)
        self.data = self.buf
        self.width = width
        self.height = height

    def from_image(image):
        buf = image.tobytes() * 2 if image.mode == 'L' else image.convert('L').tobytes() * 2
        return ImageMatrix(image.width, image.height, buf)

    def to_image(self):
        return Image.frombuffer('L', (self.width, self.height), self.buf, 'raw', 'L', 0, 1)

libsim.edge_filter.restype = None
libsim.edge_filter.argtypes = [ctypes.POINTER(ImageMatrix), ImageMatrix]

code_map = Image.open("code_map.pbm");
camera_image = code_map.rotate(30).crop((500, 500, 530, 530))

camera_matrix = ImageMatrix.from_image(camera_image)
dst_matrix = ImageMatrix(30, 30)

libsim.edge_filter(ctypes.byref(dst_matrix), camera_matrix)
camera_image.resize((300, 300)).show()
camera_image.filter(ImageFilter.FIND_EDGES).resize((300,300)).show()
dst_matrix.to_image().resize((280, 280)).show()
print(dst_matrix.height)

