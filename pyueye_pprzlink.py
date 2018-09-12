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
from os import getenv, path, makedirs
import sys
import time
import cv2
import numpy as np
import pyexiv2

from pyueye import ueye

# if PAPARAZZI_HOME not set, try PPRZLINK_DIR
PPRZLINK_DIR = getenv("PAPARAZZI_HOME", getenv("PPRZLINK_DIR"))
if PPRZLINK_DIR is not None:
    sys.path.append(PPRZLINK_DIR + "/var/lib/python")
    from pprzlink.message import PprzMessage
else:
    print("Pprzlink not found")
    sys.exit(1)


class uEyePprzlink:
    def __init__(self, verbose=False, output_dir='images'):
        self.new_msg = False
        self.idx = 0
        self.last_msg = None
        self.verbose = verbose
        self.output_dir = output_dir
        if not path.exists(self.output_dir):
            makedirs(self.output_dir)

        # camera class to simplify uEye API access
        self.verbose_print("Start uEye interface")

        self.cam = Camera()
        self.cam.init()
        check(ueye.is_SetExternalTrigger(self.cam.handle(), ueye.IS_SET_TRIGGER_SOFTWARE))
        self.cam.set_colormode(ueye.IS_CM_BGR8_PACKED)
        self.cam.set_aoi(0,0, 2048, 2048)

        # pixel clock
        self.verbose_print("Pixel clock")
        self.cam.get_pixel_clock_range(self.verbose)
        self.cam.set_pixel_clock(20)
        self.cam.get_pixel_clock(self.verbose)

        # set expo
        self.verbose_print("Expo")
        self.cam.get_exposure_range(self.verbose)
        self.cam.set_exposure(1.)
        self.cam.get_exposure(self.verbose)

        self.verbose_print("Init done")
        self.buff = self.cam.alloc_single()
        check(ueye.is_SetDisplayMode(self.cam.handle(), ueye.IS_SET_DM_DIB))
        self.verbose_print("Alloc done")

    def stop(self):
        self.cam.free_single(self.buff)
        self.verbose_print("Free mem done")
        self.cam.exit()
        self.verbose_print("leaving")

    def verbose_print(self, text):
        if self.verbose:
            print(text)

    def process_msg(self, ac_id, msg):
        self.new_msg = True
        self.idx = int(msg['photo_nr'])
        self.last_msg = msg

    def set_gps_exif(self, file_name, lat, lon, alt):
        """Adds GPS position as EXIF metadata
        file_name -- image file
        lat -- latitude (1e7 deg, as int)
        lon -- longitude (1e7 deg, as int)
        alt -- altitude MSL (in mm, as int)
        """
        def get_loc(value, loc):
            if value < 0:
                return loc[0]
            elif value > 0:
                return loc[1]
            else:
                return ""

        exiv_lat = (pyexiv2.Rational(lat, 10000000), pyexiv2.Rational(0, 1), pyexiv2.Rational(0, 1))
        exiv_lng = (pyexiv2.Rational(lon, 10000000), pyexiv2.Rational(0, 1), pyexiv2.Rational(0, 1))
        if alt > 0.:
            exiv_alt = pyexiv2.Rational(alt, 1000)
        else:
            exiv_alt = pyexiv2.Rational(0,1)

        exiv_image = pyexiv2.ImageMetadata(file_name)
        exiv_image.read()
        exiv_image["Exif.GPSInfo.GPSLatitude"] = exiv_lat
        exiv_image["Exif.GPSInfo.GPSLatitudeRef"] = get_loc(lat, ["S", "N"])
        exiv_image["Exif.GPSInfo.GPSLongitude"] = exiv_lng
        exiv_image["Exif.GPSInfo.GPSLongitudeRef"] = get_loc(lon, ["W", "E"])
        exiv_image["Exif.GPSInfo.GPSAltitude"] = exiv_alt
        exiv_image["Exif.GPSInfo.GPSAltitudeRef"] = '0'
        exiv_image["Exif.Image.GPSTag"] = 654
        exiv_image["Exif.GPSInfo.GPSMapDatum"] = "WGS-84"
        exiv_image["Exif.GPSInfo.GPSVersionID"] = '2 0 0 0'
        exiv_image.write()

    def process_image(self, image_data, file_type='jpg'):
        # reshape the image data as 1dimensional array
        image = image_data.as_1d_image()
        lat = int(self.last_msg['lat'])
        lon = int(self.last_msg['lon'])
        alt = int(self.last_msg['hmsl'])
        phi = int(self.last_msg['phi'])
        theta = int(self.last_msg['theta'])
        psi = int(self.last_msg['psi'])
        time = int(self.last_msg['itow'])
        image_name = "img_{:04d}_{:d}_{:d}_{:d}_{:d}_{:d}_{:d}_{:d}.jpg".format(
                self.idx, lat, lon, alt, phi, theta, psi, time)
        image_name = path.join(self.output_dir, image_name)
        cv2.imwrite(image_name, image)
        # also set GPS pos in exif metadata
        self.set_gps_exif(image_name, lat, lon, alt)
        self.verbose_print("save image: {}".format(image_name))


class uEyeIvy(uEyePprzlink):
    def __init__(self, verbose=False):
        from pprzlink.ivy import IvyMessagesInterface

        # init Ivy interface
        self.pprzivy = IvyMessagesInterface("pyueye")
        # init cam related part
        uEyePprzlink.__init__(self, verbose)
        # bind to message
        self.pprzivy.subscribe(self.process_msg, PprzMessage("telemetry", "DC_SHOT"))

    def __exit__(self):
        if self.pprzivy is not None:
            self.stop()

    def stop(self):
        self.pprzivy.shutdown()
        self.pprzivy = None
        uEyePprzlink.stop(self)

    def run(self):
        try:
            while True:
                if self.new_msg:
                    ret = self.cam.freeze_video(True)
                    if ret == ueye.IS_SUCCESS:
                        self.verbose_print("Freeze done")
                        img = ImageData(self.cam.handle(), self.buff)
                        self.process_image(img, 0)
                        self.verbose_print("Process done")
                    else:
                        self.verbose_print('Freeze fail with {%d}' % ret)
                    self.new_msg = False
                else:
                    time.sleep(0.1)
        except (KeyboardInterrupt, SystemExit):
            pass


class uEyeSerial(uEyePprzlink):
    def __init__(self, verbose=False):
        from pprzlink.serial import SerialMessagesInterface

        # init Serial interface
        self.pprzserial = SerialMessagesInterface(self.msg_cb)
        # init cam related part
        uEyePprzlink.__init__(self, verbose)
        # start serial thread
        self.pprzserial.start()

    def __exit__(self):
        if self.pprzserial.running:
            self.stop()

    def stop(self):
        self.pprzserial.shutdown()
        uEyePprzlink.stop(self)

    def msg_cb(self, s, m):
        if m.name() == 'DC_SHOT':
            self.process_msg(s, m)

    def run(self):
        try:
            while True:
                if self.new_msg:
                    ret = self.cam.freeze_video(True)
                    if ret == ueye.IS_SUCCESS:
                        self.verbose_print("Freeze done")
                        img = ImageData(self.cam.handle(), self.buff)
                        self.process_image(img, 0)
                        self.verbose_print("Process done")
                    else:
                        self.verbose_print('Freeze fail with {%d}' % ret)
                    self.new_msg = False
                else:
                    time.sleep(0.1)
        except (KeyboardInterrupt, SystemExit):
            pass


class uEyeKeyboard(uEyePprzlink):
    def __init__(self, verbose=False):
        # init cam related part
        uEyePprzlink.__init__(self, verbose)
        # build fake message
        msg = PprzMessage("telemetry", "DC_SHOT")
        msg['photo_nr'] = 0
        self.process_msg(0, msg)

    def __exit__(self):
        pass

    def stop(self):
        uEyePprzlink.stop(self)

    def run(self):
        try:
            while True:
                key = raw_input("Waiting enter (q+enter to leave): ")
                if key == 'q':
                    break
                ret = self.cam.freeze_video(True)
                if ret == ueye.IS_SUCCESS:
                    self.verbose_print("Freeze done")
                    img = ImageData(self.cam.handle(), self.buff)
                    self.process_image(img, 0)
                    self.verbose_print("Process done")
                    self.idx += 1
                else:
                    self.verbose_print('Freeze fail with {%d}' % ret)
        except (KeyboardInterrupt, SystemExit):
            pass

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="uEye over PPRZLINK interface")
    parser.add_argument('interface', choices=['ivy', 'serial', 'keyboard'], help="Select the interface to trigger images")
    # TODO aperture, ...
    #parser.add_argument('-a', '--aperture', dest='aperture', default=auto, help="aperture")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    if args.interface == 'ivy':
        cam_ueye = uEyeIvy(verbose=args.verbose)
    elif args.interface == 'serial':
        cam_ueye = uEyeSerial(verbose=args.verbose)
    elif args.interface == 'keyboard':
        cam_ueye = uEyeKeyboard(verbose=args.verbose)
    else:
        print("unknown interface")
        exit(1)

    cam_ueye.run()
    cam_ueye.stop()

