import ctypes
from PIL import Image

libsim = ctypes.CDLL("build/libsimulation.so")

BitMatrix32 = ctypes.c_uint * 32


class Vector2f(ctypes.Structure):
    _fields_ = [('x', ctypes.c_float), ('y', ctypes.c_float)]


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
        #libsim.imf_convert_uint8_to_int16(image_matrix)
        return image_matrix

    def to_image(self):
        #libsim.imf_normalize(self)
        #libsim.imf_convert_int16_to_uint8(self)
        image = Image.frombuffer('L', (self.n_cols, self.n_rows), self.buf,
                                 'raw', 'L', 0, 1)
        return image

    def print(self):
        libsim.print_image_matrix(self)

    def show(self, scale=10):
        self.to_image().resize(
            (int(self.n_cols * scale), int(self.n_rows * scale))).show()


libsim.cmf_estimate_rotation.restype = Vector2f
