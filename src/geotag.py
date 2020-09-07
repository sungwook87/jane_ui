#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import message_filters
import os
import sys
import rospy
import cv2

from PIL import Image
import piexif

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix

from fractions import Fraction
from datetime import datetime

# Constants that may change in multiple places
MSG_QUEUE_MAXLEN = 50

class mapping_hexa:

  def __init__(self):
    self.bridge = CvBridge()

    # Me suscribo al tópico donde se envian las imágenes
    self.image_sub = message_filters.Subscriber("/camera/image", Image)
    # Me suscribo al tópico donde se envian los datos de odometría
    self.gps_sub   = message_filters.Subscriber("/mavros/global_position/global", NavSatFix)

    # Creo el objeto que sincroniza los mensajes recibidos
    self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.gps_sub],
            queue_size=MSG_QUEUE_MAXLEN,
            slop=1)
    self.synchronizer.registerCallback(self.make_image)

  def rostime2floatSecs(self, rostime):
    return rostime.secs + (rostime.nsecs / 1000000000.0)

  def to_deg(self, value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
      loc_value = loc[0]
    elif value > 0:
      loc_value = loc[1]
    else:
      loc_value = ""
    abs_value = abs(value)
    deg = int(abs_value)
    t1 = (abs_value - deg) * 60
    min = int(t1)
    sec = round((t1 - min) * 60, 5)
    return (deg, min, sec, loc_value)

  def change_to_rational(self, number):
    """convert a number to rantional
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return (f.numerator, f.denominator)

  def set_gps_location(self, file_name, lat, lng, altitude, gpsTime):
    """Adds GPS position as EXIF metadata
    Keyword arguments:
    file_name -- image file
    lat -- latitude (as float)
    lng -- longitude (as float)
    altitude -- altitude (as float)
    """
    lat_deg = self.to_deg(lat, ["S", "N"])
    lng_deg = self.to_deg(lng, ["W", "E"])

    exiv_lat = (self.change_to_rational(lat_deg[0]), self.change_to_rational(lat_deg[1]), self.change_to_rational(lat_deg[2]))
    exiv_lng = (self.change_to_rational(lng_deg[0]), self.change_to_rational(lng_deg[1]), self.change_to_rational(lng_deg[2]))

    gps_ifd = {
      piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
      piexif.GPSIFD.GPSAltitudeRef: 0,
      piexif.GPSIFD.GPSAltitude: self.change_to_rational(round(altitude)),
      piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
      piexif.GPSIFD.GPSLatitude: exiv_lat,
      piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
      piexif.GPSIFD.GPSLongitude: exiv_lng,
      piexif.GPSIFD.GPSDateStamp: gpsTime
    }

    exif_dict = {"GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, file_name)

  def make_image(self, img_data, gps_data):
    # log some info about the image topic
    rospy.loginfo("img time_ %i", img_data.header.stamp.secs)
    rospy.loginfo("gps time: %i", gps_data.header.stamp.secs)

    try:
      cv_img = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as e:
      print(e)

    output_dir = "../img/"
    jpg_quality = 100
    compr = cv2.IMWRITE_JPEG_QUALITY;
    quality = jpg_quality  # jpg quality is in [0,100] range, png [0,9]
    params = [compr, quality]

    print(gps_data.header.stamp)
    print(gps_data.latitude)
    picGpsTimeFlt = self.rostime2floatSecs(gps_data.header.stamp)
    picNameTimeString = datetime.utcfromtimestamp(picGpsTimeFlt).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    picSaveName = ""
    picSaveName = picNameTimeString + ".jpg"
    print("save image ")
    picPath = os.path.join(output_dir, picSaveName)
    cv2.imwrite(picPath, cv_img, params)

    picGpsTimeFlt =self.rostime2floatSecs(gps_data.header.stamp)
    gpsTimeString = datetime.utcfromtimestamp(picGpsTimeFlt).strftime('%Y:%m:%d %H:%M:%S')
    self.set_gps_location(picPath, lat=gps_data.latitude, lng=gps_data.longitude,
                     altitude=gps_data.altitude, gpsTime=gpsTimeString)


def main(args):
  ic = mapping_hexa()
  rospy.init_node('mapping_hexa', anonymous=True)
  print("Tag !")

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
