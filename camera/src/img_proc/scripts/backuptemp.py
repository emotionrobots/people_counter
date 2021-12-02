#!/usr/bin/python3
#===========================================================================
#
#  img_proc_node.py
#
#  Copyright (C) 2020, E-Motion, Inc - All Rights Reserved
#  Unauthorized copying of this file, via any medium is
#  strictly prohibited
#
#  Proprietary and confidential
#  Written by Larry Li <larry@e-motion.ai>
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
#
#===========================================================================

import sys , getopt
import rospy
import cv2
import math
import json 
from datetime import datetime
import queue
import numpy as np
import sensor_msgs.point_cloud2 as pc2 
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import String 
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from img_proc.cfg import img_procConfig
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import paho.mqtt.client as mqtt
from BlobTracker import BlobTracker

node = None
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    client.subscribe("topic1")

def on_message(client, userdata, msg):
    print("Message received-> " + msg.topic + " " + str(msg.payload))
    
def getDateTime(now):
    datetime = now.strftime("%Y/%m/%d %H")
    time = format(now.strftime("%H"))
    day = format(now.strftime("%d"))
    month = format(now.strftime("%m"))
    year = now.strftime("%Y")
    weekday = now.strftime("%w")
    return datetime, time, day, month, year, weekday
    
def format(string):
    if string[0] == "0":
      return string[1:]
    return string

client.on_connect = on_connect
client.on_message = on_message

# uncomment this line!!!
client.connect("pplcnt-mqtt.e-motion.ai")

class Message:
  def __init__(self, device, deviceid, longitude, latitude, location, datetime, time, day, month,
               year, weekday, enter, exit):
    self.device = device
    self.deviceid = deviceid
    self.longitude = longitude
    self.latitude = latitude
    self.location = location
    self.datetime = datetime
    self.time = time
    self.day = day
    self.month = month
    self.year = year
    self.weekday = weekday
    self.enter = enter
    self.exit = exit

  def dictStr(self):
    d = {}
    d["device"] = self.device
    d["deviceid"] = self.deviceid
    d["longitude"] = self.longitude
    d["latitude"] = self.latitude
    d["location"] = self.location
    d["datetime"] = self.datetime
    d["time"] = self.time
    d["day"] = self.day
    d["month"] = self.month
    d["year"] = self.year
    d["weekday"] = self.weekday
    d["enter"] = self.enter
    d["exit"] = self.exit
    return json.dumps(d)

#===========================================================================
#  dynamic_reconfigure callback 
#===========================================================================
def dr_callback(config, level):
  return config


#===========================================================================
# Process front-facing cam635
#===========================================================================
class ImgProcNode(object):
   
  #===================================================
  # Constructor 
  #===================================================
  def __init__(self):

    rospy.init_node('img_proc', anonymous=False)

    self.camera = {}
    self.camera['amp'] = None
    self.camera['depth'] = None
    self.camera['x'] = None
    self.camera['y'] = None
    self.camera['z'] = None
    self.camera['chip_id'] = None
    self.camera['wafer_id'] = None

    # Setup transform listener 
    self.xform_buf = tf2_ros.Buffer()
    self.xform_listener = tf2_ros.TransformListener(self.xform_buf)

    # Support ROS message to CV image conversion 
    self.br = CvBridge()

    #mask covering the background
    self.bgmask = None
    self.aimg1 = None

    # Background learning params
    self.learningRateMax = .0001
    self.learningRateAlpha = .0001

    #OpenCV Background Subtractor
    self.bgSubtractor = cv2.createBackgroundSubtractorMOG2(varThreshold = 2, detectShadows = False)

    # Subscribe to camera data 
    rospy.Subscriber('/espros_tof_cam635/camera/image_raw1', Image, self.amp_callback)
    rospy.Subscriber('/espros_tof_cam635/camera/image_raw2', Image, self.depth_callback)
    rospy.Subscriber('/espros_tof_cam635/camera/points', PointCloud2, self.pc_callback)

    return

  #===================================================
  #  Extract amplitude or depth array from incoming
  #  message
  #===================================================
  def getArray(self, msg):
    a = self.br.imgmsg_to_cv2(msg, desired_encoding='mono16')
    a = cv2.flip(a, -1) 
    return a 

  #===================================================
  #  Extract X, Y, Z arrays from incoming point cloud 
  #  message; adjust to common global reference frame 
  # 
  #  Returns x array, y array and z array
  #===============s====================================
  def getXYZArrays(self, msg):
    gen = pc2.read_points(msg, field_names = ('x','y','z'), skip_nans=False)
    points = np.array([p for p in gen]) # p is (x,y,z) tuple
    points = points.reshape(-1, 160, 3) # row,col
    self.camera['x'] = cv2.flip(points[:,:,0], -1)
    self.camera['y'] = cv2.flip(points[:,:,1], -1)
    self.camera['z'] = cv2.flip(points[:,:,2], -1)
    return 

  #===================================================
  # Scale image 
  #===================================================
  def scaleImage(self, img, factor):
    w = int(img.shape[1] * factor)
    h = int(img.shape[0] * factor)
    dim = (w, h)
    newImg = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return newImg

  #===================================================
  # Prepare array for display by scaling and 
  # normalizing, and filtering
  #===================================================
  def prepare(self, img, scale):
    img = self.scaleImage(img, scale)
    img = cv2.normalize(img, None, 0, 65535 , cv2.NORM_MINMAX, cv2.CV_16U)
    img = cv2.medianBlur(img, 5)
    h,w = img.shape[:2]
    center = (w / 2, h / 2)
    #print(center)
    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    rotated180 = cv2.warpAffine(img, M, (w, h))
    return rotated180

  #===================================================
  #  Process camera amp image
  #===================================================
  def amp_callback(self, msg):
    self.camera['amp'] = self.getArray(msg)
    #print("DEBUG:====== amp-camera  ==============")
    #print(self.camera)
    return

  #===================================================
  #  Process camera depth image
  #===================================================
  def depth_callback(self, msg):
    self.camera['depth'] = self.getArray(msg)
    return

  #===================================================
  #  Process camera point cloud 
  #===================================================
  def pc_callback(self, msg):
    self.getXYZArrays(msg)
    return

  #===================================================
  # Remove noise with morphology opening
  #===================================================
  def morph_clean(self, bimg):
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(5,5))
    out = cv2.morphologyEx(bimg, cv2.MORPH_OPEN, kernel) 
    return out

  #===================================================
  #  Compute learningRate based on movement
  #===================================================
  def computeLearningRate(self, diff):
    lr = 0
    alpha = self.learningRateAlpha
    lrMax = self.learningRateMax
    if diff < 96:
    	eta = diff / 9600.0
    	lr = alpha / (eta + alpha/lrMax)
    return lr

  #===================================================
  #  Generate Mask Covering Background
  #===================================================
  def getBgMask(self, aimg):
    if aimg is not None:
      if self.bgmask is None:
        learningRate = 0.001
        self.aimg1 = aimg
      else:
        abs_diff = np.sum(cv2.absdiff(self.aimg1, aimg))/255.0
        learningRate = self.computeLearningRate(abs_diff)
      self.bgmask = self.bgSubtractor.apply(aimg, learningRate)
      self.bgmask = cv2.compare(self.bgmask, 0, cv2.CMP_GT)
      self.bgmask = self.morph_clean(self.bgmask)
    return self.bgmask

  #===================================================
  #  Periodic call to refresh image and compute fgf&bg and such
  #===================================================
  def periodic(self):
    dimg = self.camera['depth']
    aimg = self.camera['amp']
    zpoints = self.camera['z']
    
    backgroundMask = self.getBgMask(aimg)
    foreground = cv2.bitwise_and(aimg, aimg, mask = backgroundMask)

    if zpoints is not None:
      
      cv2.imshow('aimg',self.prepare(aimg,4))
      cv2.imshow('foreground',self.prepare(foreground,4))


  #===================================================
  #  Start processing 
  #===================================================
  def start(self):
    while not rospy.is_shutdown():
      self.periodic()
      key = cv2.waitKey(33)
      if (key & 0xFF) == ord('q'):
        break 
    return


#===========================================================================
#  Main() 
#===========================================================================
def main(argv):
  global node
  node = ImgProcNode()
  srv = Server(img_procConfig, dr_callback)
  node.start()
  return

if __name__=='__main__':
  main(sys.argv[1:])    
