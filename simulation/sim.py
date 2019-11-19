#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter, ImageDraw
from sim_wrapper import *

Image.MAX_IMAGE_PIXELS = 1000000000
n = 30

code_map = Image.open("code_map.pbm").convert('L').crop((0, 0, 1000, 1000))
#code_map.crop((500, 500, 500 + n, 500 + n)).resize((300, 300)).show()


def plot_scale_estimates(plt, code_map, n, x, y):
    camera_image = code_map.crop((x, y, x + n, y + n))
    scale_range = [i / 10 for i in range(10, 100)]
    scale_estimates = []
    for scale in scale_range:
        scaled_image = camera_image.resize((int(n * scale), int(n * scale)))
        camera_matrix = ImageMatrix.from_image(scaled_image)
        scale_estimate = libsim.img_estimate_scale(camera_matrix)
        if scale_estimate > 70:
            print('bad', x, y)
            camera_image.show()
        scale_estimates.append(scale_estimate)
    plt.plot(scale_range, scale_estimates)


k = 45
for rot in [k, 90 + k, 180 + k, 270 + k]:
    rotated_map = code_map.rotate(rot, Image.BILINEAR)
    for x in range(250, 750 - n, n):
        for y in range(250, 750 - n, n):
            plot_scale_estimates(plt, rotated_map, n, x, y)
            print(rot, x, y)
plt.plot([0], [0])
plt.show()

exit()

rotated_bit_mask = BitMatrix32()
libsim.rotation_bit_mask(rotated_bit_mask, camera_matrix, rotation)

for row in rotated_bit_mask:
    libsim.print_bits(row, 32)

exit()

test_values = []
for i in range(30):
    test_image = Image.new('L', (30, 30), 0)
    draw = ImageDraw.Draw(test_image)
    draw.line(((0, i), (29, 29 - i)), 255, 2)
    draw.line(((i, 30), (29 - i, 0)), 255, 2)
    test_matrix = ImageMatrix.from_image(test_image)
    rotation_vector = libsim.estimate_rotation(test_matrix)
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
    rotation_vector = libsim.estimate_rotation(test_matrix)
    test_values.append(
        math.atan2(rotation_vector.y, rotation_vector.x) * 180 / math.pi)
plt.plot(range(-45, 45), test_values)

plt.plot(range(-45, 45), range(-45, 45))
#plt.show()
