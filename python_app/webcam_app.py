#!/usr/bin/env python3
import cv2
import numpy as np
from codemap.webcam import WebCamLocalization

window_name = "Webcam Pipeline"
cv2.namedWindow(window_name,
                flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                | cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow(window_name, (3 * 256, 2 * 256))

libcodemap = np.ctypeslib.load_library('libcodemap', 'build')
webcam_loc = WebCamLocalization(libcodemap, 0)

while webcam_loc.update():
    # print results
    loc_msg = webcam_loc.loc_ctx.get_location_msg()
    for key in ('odometry', 'location', 'correlation'):
        print(f'{key.title()}\t{loc_msg[key]}')
    print()
    # display image
    cv2.imshow(window_name, webcam_loc.loc_ctx.get_pipeline_montage())
    key = cv2.waitKey(1)
    # exit on esc key
    if key == 27:
        break
