#!/usr/bin/env python3
import cv2
from codemap.webcam import WebCamLocalization

window_name = "Webcam Pipeline"
cv2.namedWindow(window_name,
                flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                | cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow(window_name, (3 * 256, 2 * 256))

webcam_loc = WebCamLocalization(0)
while webcam_loc.update():
    # print results
    webcam_loc.loc_ctx.print()
    print()
    # display image
    cv2.imshow(window_name, webcam_loc.loc_ctx.get_pipeline_montage())
    key = cv2.waitKey(1)
    # exit on esc key
    if key == 27:
        break
