#!/usr/bin/env python3

import tkinter, math, ctypes
from PIL import Image, ImageTk, ImageDraw
from sim_wrapper import *

class CodeMapCanvas():
    def __init__(self,
                 parent,
                 code_map_image,
                 size,
                 camera_size,
                 camera_resolution,
                 scale,
                 update_callback=None):
        if code_map_image.width < size[0] or code_map_image.height < size[1]:
            raise AssertionError(
                "CodeMapCanvas size is greater than the size of map image")
        if size[0] < camera_size[0] or size[0] < camera_size[0]:
            raise AssertionError(
                "CodeMapCanvas size is less than the size of camera view")
        self.scale = scale
        self.camera_size = camera_size
        self.camera_resolution = camera_resolution
        self.camera_corners = (
            (camera_size[0] * scale / 2, camera_size[1] * scale / 2),
            (camera_size[0] * scale / 2, -camera_size[1] * scale / 2),
            (-camera_size[0] * scale / 2, -camera_size[1] * scale / 2),
            (-camera_size[0] * scale / 2, camera_size[1] * scale / 2),
        )
        self.camera_rotation = 0
        self.code_map_image = code_map_image
        self.canvas = tkinter.Canvas(parent,
                                     width=size[0] * scale,
                                     height=size[1] * scale)
        self.view_box = [0, 0, size[0], size[1]]
        self.image_area = self.canvas.create_image(0, 0, anchor=tkinter.NW)
        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<4>", self.on_wheel)
        self.canvas.bind("<5>", self.on_wheel)
        self.canvas.configure(cursor="hand1")
        self.update_callback = update_callback
        self.update_view()

    def update_view(self):
        if self.view_box[0] < 0:
            self.view_box[2] -= self.view_box[0]
            self.view_box[0] = 0
        elif self.view_box[2] >= self.code_map_image.width:
            self.view_box[0] -= (self.view_box[2] -
                                 (self.code_map_image.width - 1))
            self.view_box[2] = self.code_map_image.width - 1
        if self.view_box[1] < 0:
            self.view_box[3] -= self.view_box[1]
            self.view_box[1] = 0
        elif self.view_box[3] >= self.code_map_image.height:
            self.view_box[1] -= (self.view_box[3] -
                                 (self.code_map_image.height - 1))
            self.view_box[3] = self.code_map_image.height - 1
        unscaled_width = self.view_box[2] - self.view_box[0]
        unscaled_height = self.view_box[3] - self.view_box[1]
        width = round(unscaled_width * self.scale)
        height = round(unscaled_height * self.scale)
        view_image = code_map_image.crop((round(val) for val in self.view_box))
        display_image = view_image.resize((width, height)).convert('RGB')
        angle = self.camera_rotation * math.pi / 180
        rotated_camera_corners = [((math.cos(angle) * x + math.sin(angle) * y),
                                   (math.cos(angle) * y - math.sin(angle) * x))
                                  for x, y in self.camera_corners]
        rotated_camera_corners.append(rotated_camera_corners[0])
        draw = ImageDraw.Draw(display_image)
        for i in range(len(rotated_camera_corners) - 1):
            y1, x1 = rotated_camera_corners[i]
            y2, x2 = rotated_camera_corners[i + 1]
            draw.line((x1 + width / 2, y1 + height / 2, x2 + width / 2,
                       y2 + height / 2),
                      fill=(0, 255, 0),
                      width=20)
        self.code_map_view = ImageTk.PhotoImage(image=display_image)
        self.canvas.itemconfig(self.image_area, image=self.code_map_view)
        camera_image = view_image.convert('L').resize((
            round(unscaled_width * self.camera_resolution[0] /
                  self.camera_size[0]),
            round(unscaled_height * self.camera_resolution[1] /
                  self.camera_size[1]),
        ), Image.BILINEAR)
        crop_box = [
            round((camera_image.width - self.camera_resolution[0]) / 2),
            round((camera_image.height - self.camera_resolution[1]) / 2),
            round((camera_image.width + self.camera_resolution[0]) / 2),
            round((camera_image.height + self.camera_resolution[1]) / 2),
        ]
        camera_image = camera_image.convert('L').rotate(
            self.camera_rotation, Image.BILINEAR).crop(crop_box)
        if self.update_callback:
            self.update_callback(camera_image)

    def on_click(self, event):
        self.last_event = event

    def on_drag(self, event):
        dx = (event.x - self.last_event.x) / self.scale
        dy = (event.y - self.last_event.y) / self.scale
        self.last_event = event
        self.view_box[0] -= dx
        self.view_box[1] -= dy
        self.view_box[2] -= dx
        self.view_box[3] -= dy
        self.update_view()

    def on_wheel(self, event):
        if event.num == 4:
            self.camera_rotation += 3
        elif event.num == 5:
            self.camera_rotation -= 3
        self.update_view()

    def on_key(self, event):
        if event.char == 'q':
            self.camera_rotation -= 2
        elif event.char == 'e':
            self.camera_rotation += 2
        elif event.char == 'w':
            self.view_box[1] -= 1
            self.view_box[3] -= 1
        elif event.char == 'a':
            self.view_box[0] -= 1
            self.view_box[2] -= 1
        elif event.char == 's':
            self.view_box[1] += 1
            self.view_box[3] += 1
        elif event.char == 'd':
            self.view_box[0] += 1
            self.view_box[2] += 1
        self.update_view()


class ImagePipelineCanvas:
    def __init__(self, parent, scale, image_size, update_callback=None):
        self.canvas = tkinter.Canvas(root,
                                     width=image_size[0] * scale,
                                     height=image_size[1] * scale)
        self.image_area = self.canvas.create_image(0, 0, anchor=tkinter.NW)
        self.scale = scale
        self.update_callback = update_callback

    def on_image_update(self, image):
        self.image = ImageTk.PhotoImage(
            image=image.resize((image.width * self.scale,
                                image.height * self.scale)))
        self.canvas.itemconfig(self.image_area, image=self.image)
        if self.update_callback:
            self.update_callback(image)


class ImageProcessor:
    def __init__(self, bits_per_pixel, update_callback=None):
        self.update_callback = update_callback
        self.bits_per_pixel = bits_per_pixel

    def process_image(self, image):
        matrix = ImageMatrix.from_image(image)
        rotation = libsim.cmf_estimate_rotation(matrix)
        rotation.x *= self.bits_per_pixel
        rotation.y *= -self.bits_per_pixel
        unrotated_matrix = ImageMatrix(32, 32)
        libsim.imf_rotate(unrotated_matrix, matrix, rotation)
        libsim.imf_threshold(unrotated_matrix, 127)
        image = unrotated_matrix.to_image()
        if self.update_callback:
            self.update_callback(image)


code_map_image = Image.open("code_map.pbm")
view_size = (32, 32)
camera_resolution = (30, 30)
camera_size = (23, 23)
scale = 10

root = tkinter.Tk()
root.title("CodeMap Sim")

filtered_canvas = ImagePipelineCanvas(root, scale, view_size)
image_filter = ImageProcessor(camera_size[0] / camera_resolution[0],
                              filtered_canvas.on_image_update)
camera_view_canvas = ImagePipelineCanvas(root, scale, camera_resolution,
                                         image_filter.process_image)
code_map_canvas = CodeMapCanvas(root, code_map_image, view_size, camera_size,
                                camera_resolution, scale,
                                camera_view_canvas.on_image_update)

root.bind("<Escape>", lambda e: root.destroy())
for key in ('<q>', '<e>', '<w>', '<a>', '<s>', '<d>'):
    root.bind(key, code_map_canvas.on_key)

code_map_canvas.canvas.grid(row=0, column=0)
camera_view_canvas.canvas.grid(row=0, column=1)
filtered_canvas.canvas.grid(row=0, column=2)

root.mainloop()
