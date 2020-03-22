import ctypes
import numpy as np

libcodemap = ctypes.CDLL("build/libcodemap.so")

# bitwise_utils.h
BitMatrix32 = ctypes.c_uint * 32
BitMatrix64 = ctypes.c_ulonglong * 64


# math_utils.h
class Vector2f(ctypes.Structure):
    _fields_ = [('x', ctypes.c_float), ('y', ctypes.c_float)]


class Matrix2f(ctypes.Structure):
    _fields_ = [('a', ctypes.c_float), ('b', ctypes.c_float),
                ('c', ctypes.c_float), ('d', ctypes.c_float)]


# image_utils.h
class ImagePoint(ctypes.Structure):
    _fields_ = [('x', ctypes.c_short), ('y', ctypes.c_short)]


class ImageMatrix(ctypes.Structure):
    _fields_ = [('data', ctypes.c_void_p), ('size', ImagePoint)]

    def __init__(self, np_array):
        self.data = np_array.ctypes.data_as(ctypes.c_void_p)
        self.size.y, self.size.x = np_array.shape


# mls_query.h
class MlsIndex(ctypes.Structure):
    _fields_ = [
        ('sequence', ctypes.POINTER(ctypes.c_uint)),
        ('sorted_code_positions', ctypes.POINTER(ctypes.c_ushort)),
        ('sequence_length', ctypes.c_ushort),
        ('code_length', ctypes.c_ubyte),
    ]


# code_extraction.h
class AxisCode32(ctypes.Structure):
    _fields_ = [
        ('bits', ctypes.c_uint),
        ('mask', ctypes.c_uint),
        ('n_errors', ctypes.c_ushort),
        ('n_samples', ctypes.c_ushort),
    ]


class AxisCode64(ctypes.Structure):
    _fields_ = [
        ('bits', ctypes.c_ulonglong),
        ('mask', ctypes.c_ulonglong),
        ('n_errors', ctypes.c_ushort),
        ('n_samples', ctypes.c_ushort),
    ]


# location_decode.h
class AxisPosition(ctypes.Structure):
    _fields_ = [
        ('center', ctypes.c_ushort),
        ('span', ctypes.c_ubyte, 6),
        ('inverted', ctypes.c_ubyte, 1),
        ('reversed', ctypes.c_ubyte, 1),
    ]


class Location(ctypes.Structure):
    _fields_ = [
        ('match_size', ctypes.c_ushort),
        ('x', ctypes.c_ushort),
        ('y', ctypes.c_ushort),
        ('direction', ctypes.c_ubyte),
    ]


class ScaleQuery(ctypes.Structure):
    _fields_ = [('row_code', AxisCode64), ('col_code', AxisCode64),
                ('lower_bound', ctypes.c_float),
                ('upper_bound', ctypes.c_float), ('step_size', ctypes.c_float)]


class ScaleMatch(ctypes.Structure):
    _fields_ = [('location', Location), ('row_code', AxisCode32),
                ('col_code', AxisCode32), ('scale', ctypes.c_float)]


class OutlierFilter(ctypes.Structure):
    _fields_ = [('filtered_match', ScaleMatch),
                ('distance_threshold', ctypes.c_ushort),
                ('match_size_threshold', ctypes.c_ushort),
                ('bit_error_ratio_threshold', ctypes.c_ubyte),
                ('max_rejection_count', ctypes.c_ubyte),
                ('rejection_count', ctypes.c_ubyte)]


# visual_odometry.h
class Correlation(ctypes.Structure):
    _fields_ = [('image', ImageMatrix), ('buffer', ImageMatrix),
                ('translation', Vector2f),
                ('squared_magnitude_threshold', ctypes.c_float),
                ('squared_magnitude_max', ctypes.c_float),
                ('squared_magnitude_sum', ctypes.c_float)]


class VisualOdometry(ctypes.Structure):
    _fields_ = [
        ('correlation', Correlation),
        ('position', Vector2f),
        ('quadrant_rotation', Vector2f),
        ('quadrant_count', ctypes.c_int),
        ('step_count', ctypes.c_uint),
    ]


# localization_loop.h


class CorrelationArrays():
    def __init__(self, correlation):
        self.correlation = correlation
        correlation.image.__init__(self.image)
        correlation.buffer.__init__(self.buffer)


class LocalizationContext(ctypes.Structure):
    _fields_ = [('derotated_image', ImageMatrix),
                ('sharpened_image', ImageMatrix),
                ('binary_image', BitMatrix64), ('binary_mask', BitMatrix64),
                ('scale_query', ScaleQuery), ('scale_match', ScaleMatch),
                ('outlier_filter', OutlierFilter), ('odom', VisualOdometry),
                ('rotation_scale', ctypes.c_float),
                ('histogram', ctypes.c_uint * 256),
                ('threshold', ctypes.c_ubyte * 2),
                ('frame_count', ctypes.c_uint)]

    def __init__(self):
        self.frame_count = 0
        # create buffers
        self.derotated_image_array = np.empty((64, 64),
                                              dtype=np.ubyte,
                                              order='C')
        self.sharpened_image_array = np.empty((62, 62),
                                              dtype=np.ubyte,
                                              order='C')
        self.correlation_image_array = np.empty((32, 32),
                                                dtype=np.csingle,
                                                order='C')
        self.correlation_buffer_array = np.empty((32, 32),
                                                 dtype=np.csingle,
                                                 order='C')
        self.thresholded_image_array = np.empty((64, 64),
                                                dtype=np.ubyte,
                                                order='C')
        self.extracted_image_array = np.empty((32, 32),
                                              dtype=np.ubyte,
                                              order='C')
        # set links to buffers
        self.derotated_image.__init__(self.derotated_image_array)
        self.sharpened_image.__init__(self.sharpened_image_array)
        self.odom.correlation.image.__init__(self.correlation_image_array)
        self.odom.correlation.buffer.__init__(self.correlation_buffer_array)
        self.corr_a = self.correlation_image_array
        self.corr_b = self.correlation_buffer_array
        # setup params
        self.rotation_scale = 1.0
        self.scale_query.lower_bound = 0.8
        self.scale_query.upper_bound = 1.2
        self.scale_query.step_size = 0.02
        self.outlier_filter.distance_threshold = 200
        self.outlier_filter.match_size_threshold = 20
        self.outlier_filter.bit_error_ratio_threshold = 5
        self.outlier_filter.max_rejection_count = 10
        self.odom.correlation.squared_magnitude_threshold = 0.01

    def run(self, frame):
        # run loop
        location_updated = (1 == libcodemap.localization_loop_run(
            ctypes.byref(self), ImageMatrix(frame)))
        # create thresholded image
        libcodemap.bm64_transpose(self.binary_image)
        libcodemap.bm64_transpose(self.binary_mask)
        libcodemap.bm64_to_img(
            ctypes.byref(ImageMatrix(self.thresholded_image_array)),
            self.binary_image, self.binary_mask)
        # create extracted image
        binary_image = BitMatrix32()
        binary_mask = BitMatrix32()
        libcodemap.bm32_from_axiscodes(binary_image, binary_mask,
                                       self.scale_match.row_code,
                                       self.scale_match.col_code)
        libcodemap.bm32_to_img(
            ctypes.byref(ImageMatrix(self.extracted_image_array)),
            binary_image, binary_mask)
        # create correlation image
        self.corr_a, self.corr_b = self.corr_b, self.corr_a
        self.centered_correlation = np.roll(np.roll(self.corr_a.real, 16, 0),
                                            16, 1)
        self.centered_correlation *= 255
        self.centered_correlation /= self.odom.correlation.squared_magnitude_max
        return location_updated

    def print(self):
        libcodemap.print_odometry(ctypes.byref(self.odom))
        libcodemap.print_location_match(ctypes.byref(self.scale_match))


MLS_INDEX = MlsIndex.in_dll(libcodemap, "MLS_INDEX")
QUADRANT_LOOKUP = (Vector2f(1, 0), Vector2f(0,
                                            1), Vector2f(-1,
                                                         0), Vector2f(0, -1))
