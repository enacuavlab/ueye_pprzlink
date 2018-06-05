#!/usr/bin/env python

#------------------------------------------------------------------------------
#                 PyuEye example - main module
#
# Copyright (c) 2017 by IDS Imaging Development Systems GmbH.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#------------------------------------------------------------------------------

from pyueye_camera import Camera
from pyueye_utils import ImageData, check
from os import getenv
import sys
import time
import cv2
import numpy as np


from pyueye import ueye

# if PAPARAZZI_HOME not set, try PPRZLINK_DIR
PPRZLINK_DIR = getenv("PAPARAZZI_HOME", getenv("PPRZLINK_DIR"))
if PPRZLINK_DIR is not None:
    sys.path.append(PPRZLINK_DIR + "/var/lib/python")
    from pprzlink.ivy import IvyMessagesInterface
    from pprzlink.message import PprzMessage
else:
    print("Pprzlink not found")
    sys.exit(1)


#class uEyePprzlink:
#    def __init__(self):
#        self.take_shot = False
#        self.idx = 0
#        self.last_msg = None
#
#    def __exit__(self):
#        self.stop()
#

take_shot = False
idx = 0

def process_image(image_data, num):
    # reshape the image data as 1dimensional array
    image = image_data.as_1d_image()    
    cv2.imwrite("img-" + str(idx) + ".jpg", image)

def process_msg(ac_id, msg):
    global take_shot
    global idx
    take_shot = True
    lat = msg['lat']
    lon = msg['lon']
    idx = msg['photo_nr']

def main():
    global take_shot

    pprzivy = IvyMessagesInterface("pyueye")

    # camera class to simplify uEye API access
    print("Start")
    cam = Camera()
    cam.init()
    check(ueye.is_SetExternalTrigger(cam.handle(), ueye.IS_SET_TRIGGER_SOFTWARE))
    cam.set_colormode(ueye.IS_CM_BGR8_PACKED)
    cam.set_aoi(0,0, 2048, 2048)

    # pixel clock
    print("Pixel clock")
    cam.get_pixel_clock_range(True)
    cam.set_pixel_clock(20)
    cam.get_pixel_clock(True)

    # set expo
    print("Expo")
    cam.get_exposure_range(True)
    cam.set_exposure(1.)
    cam.get_exposure(True)

    print("Init done")
    buff = cam.alloc_single()
    check(ueye.is_SetDisplayMode(cam.handle(), ueye.IS_SET_DM_DIB))
    print("Alloc done")

    pprzivy.subscribe(process_msg, PprzMessage("telemetry", "DC_SHOT"))


    try:
        while True:
            if take_shot:
                ret = cam.freeze_video(True)
                if ret == ueye.IS_SUCCESS:
                    print("Freeze done")
                    img = ImageData(cam.handle(), buff)
                    process_image(img, 0)
                    print("Process done")
                else:
                    print('Freeze fail with {%d}' % ret)
                take_shot = False
            else:
                time.sleep(0.1)

            #key = raw_input("Waiting enter (q+enter to leave): ")
            #if key == 'q':
            #    break
            #ret = cam.freeze_video(True)
            #if ret == ueye.IS_SUCCESS:
            #    print("Freeze done")
            #    img = ImageData(cam.handle(), buff)
            #    process_image(img, 0)
            #    print("Process done")
            #else:
            #    print('Freeze fail with {%d}' % ret)
    except (KeyboardInterrupt, SystemExit):
        pass

    cam.free_single(buff)
    print("Free mem done")
    cam.exit()
    pprzivy.shutdown()
    print("leaving")

if __name__ == "__main__":
    main()

