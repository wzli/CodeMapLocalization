#!/usr/bin/env python3

import usb.core
from PIL import Image, ImageTk
import tkinter

class MouseSensor():
    PIX_GRAB_REG = 0x0B
    def __init__(self, vendor_id, product_id, sensor_size):
        self.sensor_size = sensor_size
        self.device = usb.core.find(idVendor=vendor_id, idProduct=product_id)
        if(self.device.is_kernel_driver_active(0)):
            self.device.detach_kernel_driver(0)
        self.device.reset()
        self.device.set_configuration()

    def read_endpoint(self):
        endpoint = self.device[0][(0,0)][0]
        return self.device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)

    def read_register(self, register, length):
        return self.device.ctrl_transfer(bmRequestType = 0xC0, bRequest = 0x01, wValue = 0x0000, wIndex = register, data_or_wLength = length)

    def write_register(self, register, value, length):
        return self.device.ctrl_transfer(bmRequestType = 0x40, bRequest = 0x01, wValue = value, wIndex = register, data_or_wLength = length)

    def grab_frame(self):
        frame = Image.new('L', (self.sensor_size, self.sensor_size), 0)
        while(mouse.write_register(MouseSensor.PIX_GRAB_REG, 0, 0) != 0):
            pass
        for row in range(self.sensor_size):
            for col in range(self.sensor_size):
                pixel = mouse.read_register(MouseSensor.PIX_GRAB_REG, 1)[0]
                while pixel < 0x80:
                    pixel = mouse.read_register(MouseSensor.PIX_GRAB_REG, 1)[0]
                frame.putpixel((col, row), (pixel & 0x7F) << 1)
        return frame


mouse = MouseSensor(vendor_id = 0x046d, product_id = 0xc019, sensor_size = 15)

while False:
    try:
        data = mouse.read_endpoint()
        print(data)
    except usb.core.USBError as e:
        print(e)

while False:
    try:
        data = mouse.read_register(0x09, 1)
        print(data)
    except usb.core.USBError as e:
        print(e)

scale = 20

root = tkinter.Tk()
root.title("Mouse Frame Grabber")
for key in ('<Escape>', '<q>', '<Q>'):
    root.bind(key, lambda e: root.destroy())

canvas = tkinter.Canvas(root, width=mouse.sensor_size * scale, height=mouse.sensor_size * scale)
canvas.pack()
image_area = canvas.create_image(0,0, anchor=tkinter.NW)
image = None

def update_frame():
    global image
    image = ImageTk.PhotoImage(image=mouse.grab_frame().resize((mouse.sensor_size * scale, mouse.sensor_size * scale)))
    canvas.itemconfig(image_area, image=image)
    root.after(100, update_frame)

root.after(0, update_frame)
root.mainloop()

