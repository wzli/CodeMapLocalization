import ctypes
from PIL import Image

libsim = ctypes.CDLL("build/libsimulation.so")

BitMatrix32 = ctypes.c_uint * 32

class AxisCode(ctypes.Structure):
    _fields_ = [('bits', ctypes.c_uint), ('mask', ctypes.c_uint)]


class Vector2f(ctypes.Structure):
    _fields_ = [('x', ctypes.c_float), ('y', ctypes.c_float)]


class ImageMatrix(ctypes.Structure):
    _fields_ = [('data', ctypes.c_char_p), ('n_cols', ctypes.c_short),
                ('n_rows', ctypes.c_short)]
    pixel_size = 1

    def __init__(self, width, height, buf=None):
        self.buf = buf if buf else bytes(ImageMatrix.pixel_size * width *
                                         height)
        self.data = self.buf
        self.n_cols = width
        self.n_rows = height

    def from_image(image):
        buf = image.tobytes() * 2 if image.mode == 'L' else image.convert(
            'L').tobytes() * 2
        image_matrix = ImageMatrix(image.width, image.height, buf)
        if ImageMatrix.pixel_size == 2:
            libsim.imf_convert_uint8_to_int16(image_matrix)
        return image_matrix

    def to_image(self):
        #libsim.imf_normalize(self)
        if ImageMatrix.pixel_size == 2:
            libsim.imf_convert_int16_to_uint8(self)
        image = Image.frombuffer('L', (self.n_cols, self.n_rows), self.buf,
                                 'raw', 'L', 0, 1).copy()
        if ImageMatrix.pixel_size == 2:
            libsim.imf_convert_uint8_to_int16(self)
        return image

    def print(self):
        libsim.print_image_matrix(self)

    def show(self, scale=10):
        self.to_image().resize(
            (int(self.n_cols * scale), int(self.n_rows * scale))).show()


libsim.cmf_estimate_rotation.restype = Vector2f

test_image = ImageMatrix(1, 8)
libsim.imf_fill(test_image, 0xFF)
while test_image.buf[ImageMatrix.pixel_size] != 0xFF:
    ImageMatrix.pixel_size += 1
if ImageMatrix.pixel_size not in (1, 2):
    raise AssertionError(f"unsupported pixel size {ImageMatrix.pixel_size}")
