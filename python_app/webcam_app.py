#!/usr/bin/env python3
import cv2
import codemap.core

window_name = "Webcam Pipeline"
cam = cv2.VideoCapture(0)

if cam.isOpened():  # try to get the first frame
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 64)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 64)
    rval, frame = cam.read()
else:
    rval = False

cv2.namedWindow(window_name,
                flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                | cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow(window_name, (3 * 256, 2 * 256))

loc_ctx = codemap.core.LocalizationContext()

while rval:
    # read frame
    rval, frame = cam.read()
    # convert to grayscale
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # crop center square
    center = frame.shape[1] // 2
    offset = frame.shape[0] // 2
    frame = frame[:, center - offset:center + offset]
    # resize to 64x64
    frame = cv2.resize(frame, (64, 64))
    # run localization
    loc_ctx.run(frame)
    # print results
    loc_ctx.print()
    print()
    # display image
    cv2.imshow(window_name, loc_ctx.get_pipeline_montage())
    key = cv2.waitKey(1)
    # exit on ESC
    if key == 27:
        break

cv2.destroyWindow(window_name)
