import ctypes
from PIL import Image

libsim = ctypes.CDLL("build/libsimulation.so")

BitMatrix32 = ctypes.c_uint * 32


class MlsIndex(ctypes.Structure):
    _fields_ = [
        ('sequence', ctypes.POINTER(ctypes.c_uint)),
        ('sorted_code_positions', ctypes.POINTER(ctypes.c_ushort)),
        ('sequence_length', ctypes.c_ushort),
        ('code_length', ctypes.c_ubyte),
    ]


class AxisCode32(ctypes.Structure):
    _fields_ = [
        ('bits', ctypes.c_uint),
        ('mask', ctypes.c_uint),
        ('n_errors', ctypes.c_ushort),
        ('n_samples', ctypes.c_ushort),
    ]


class Vector2f(ctypes.Structure):
    _fields_ = [('x', ctypes.c_float), ('y', ctypes.c_float)]


class Matrix2f(ctypes.Structure):
    _fields_ = [('a', ctypes.c_float), ('b', ctypes.c_float),
                ('c', ctypes.c_float), ('d', ctypes.c_float)]


class AxisPosition(ctypes.Structure):
    _fields_ = [
        ('start', ctypes.c_ushort),
        ('span', ctypes.c_ubyte, 6),
        ('inverted', ctypes.c_ubyte, 1),
        ('reveresd', ctypes.c_ubyte, 1),
    ]


class Location(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_ushort),
        ('y', ctypes.c_ushort),
        ('rotation', Vector2f),
        ('match_size', ctypes.c_short),
    ]


class ImagePoint(ctypes.Structure):
    _fields_ = [('x', ctypes.c_short), ('y', ctypes.c_short)]


class ImageMatrix(ctypes.Structure):
    _fields_ = [('data', ctypes.c_char_p), ('n_cols', ctypes.c_short),
                ('n_rows', ctypes.c_short)]

    def __init__(self, width, height, buf=None):
        self.buf = buf if buf else bytes(width * height)
        self.data = self.buf
        self.n_cols = width
        self.n_rows = height

    def from_image(image):
        buf = image.tobytes() if image.mode == 'L' else image.convert(
            'L').tobytes()
        image_matrix = ImageMatrix(image.width, image.height, buf)
        return image_matrix

    def to_image(self):
        image = Image.frombuffer('L', (self.n_cols, self.n_rows), self.buf,
                                 'raw', 'L', 0, 1).copy()
        return image

    def print(self):
        libsim.print_image_matrix(self)

    def show(self, scale=10):
        self.to_image().resize(
            (int(self.n_cols * scale), int(self.n_rows * scale))).show()


class ImageMatrixInt32(ctypes.Structure):
    _fields_ = [('data', ctypes.c_char_p), ('n_cols', ctypes.c_short),
                ('n_rows', ctypes.c_short)]

    def __init__(self, width, height, buf=None):
        self.buf = buf if buf else bytes(4 * width * height)
        self.data = self.buf
        self.n_cols = width
        self.n_rows = height

    def from_image(image):
        buf = image.tobytes() if image.mode == 'I' else image.convert(
            'I').tobytes()
        image_matrix = ImageMatrix(image.width, image.height, buf)
        return image_matrix

    def to_image(self):
        image = Image.frombuffer('I', (self.n_cols, self.n_rows), self.buf,
                                 'raw', 'I', 0, 1).copy()
        return image

    def show(self, scale=1):
        self.to_image().resize(
            (int(self.n_cols * scale), int(self.n_rows * scale))).show()


libsim.test_add_angle.restype = Vector2f
libsim.img_estimate_rotation.restype = Vector2f
libsim.img_estimate_scale.restype = ctypes.c_float
libsim.decode_axis_position.restype = AxisPosition
libsim.deduce_location.restype = Location

MLS_INDEX = MlsIndex.in_dll(libsim, "MLS_INDEX")
