#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter, ImageDraw
from sim_wrapper import *

n = 30

code_map = Image.open("code_map.pbm").convert('L')
code_map.crop((500, 500, 500 + n, 500 + n)).resize((300, 300)).show()
camera_image = code_map.rotate(30, Image.BILINEAR).crop(
    (500, 500, 500 + n, 500 + n))
camera_image.resize((300, 300)).show()
camera_matrix = ImageMatrix.from_image(camera_image)

edge_matrix = ImageMatrix(n, n)
libsim.imf_edge_filter(ctypes.byref(edge_matrix), camera_matrix)
#edge_matrix.print()

hough_matrix = ImageMatrix(800, 800)
libsim.imf_hough_line_transform(hough_matrix, camera_matrix)
#hough_matrix.print()

edge_hough_matrix = ImageMatrix(800, 800)
libsim.imf_normalize(edge_matrix, 255)
#libsim.threshold_elements(edge_matrix, 200, 255)
libsim.imf_hough_line_transform(edge_hough_matrix, edge_matrix)
#edge_hough_matrix.print()

rotation = libsim.cmf_estimate_rotation(camera_matrix)
print("rotation", math.atan2(rotation.y, rotation.x) * 180 / math.pi)
rotated_matrix = ImageMatrix(32, 32)
rotation.y *= -1
libsim.imf_rotate(rotated_matrix, camera_matrix, rotation, 127)

bit_mask = BitMatrix32()
bit_matrix = BitMatrix32()
libsim.cmf_bit_matrix_conversion(bit_matrix, bit_mask, rotated_matrix, 80, 160)

row_code = ctypes.c_uint()
col_code = ctypes.c_uint()
libsim.bm32_extract_codes(ctypes.byref(row_code), ctypes.byref(col_code),
                          bit_matrix, bit_mask)

#camera_image.rotate(-30, Image.BILINEAR).resize((300, 300)).show()

#camera_matrix.show()
#edge_matrix.show(10)
hough_matrix.show(1.1)
edge_hough_matrix.show(1.0)
rotated_matrix.show()

for row in bit_mask:
    libsim.print_bits(row, 32)
print('')
for row in bit_matrix:
    libsim.print_bits(row, 32)
print('')
libsim.print_bits(row_code, 32)
libsim.print_bits(col_code, 32)

exit()

test_values = []
for i in range(30):
    test_image = Image.new('L', (30, 30), 0)
    draw = ImageDraw.Draw(test_image)
    draw.line(((0, i), (29, 29 - i)), 255, 2)
    draw.line(((i, 30), (29 - i, 0)), 255, 2)
    test_matrix = ImageMatrix.from_image(test_image)
    rotation_vector = libsim.cmf_estimate_rotation(test_matrix)
    test_values.append(
        math.atan2(rotation_vector.y, rotation_vector.x) * 180 / math.pi)
x_range = [math.atan2(i - 14, 14) * 180 / math.pi for i in range(30)]
plt.plot(x_range, test_values)

test_values = []
for i in range(-45, 45):
    test_image = code_map.crop(
        (500, 500, 500 + int(n * 1.5),
         500 + int(n * 1.5))).rotate(i, Image.BILINEAR).crop(
             (int(n * 0.25), int(n * 0.25), int(n * 1.25), int(n * 1.25)))
    test_matrix = ImageMatrix.from_image(test_image)
    rotation_vector = libsim.cmf_estimate_rotation(test_matrix)
    test_values.append(
        math.atan2(rotation_vector.y, rotation_vector.x) * 180 / math.pi)
plt.plot(range(-45, 45), test_values)

plt.plot(range(-45, 45), range(-45, 45))
#plt.show()
