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
import os
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
    print("connected to server")
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
client.connect("mqtt.e-motion.ai")


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
    self.learningRateMax = .00001
    self.learningRateAlpha = .00001

    #OpenCV Background Subtractor
    self.bgSubtractor = cv2.createBackgroundSubtractorMOG2(varThreshold = 2, detectShadows = False)

    # blob tracker
    self.tracker = BlobTracker()
    self.enterExit = []

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
    #img = cv2.medianBlur(img, 5)
    h,w = img.shape[:2]
    center = (w / 2, h / 2)
    #print(center)
    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    rotated180 = cv2.warpAffine(img, M, (w, h))
    return rotated180*2

  #===================================================
  # Remove noise with morphology opening
  #===================================================
  def morph_clean(self, bimg):
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))
    open = cv2.morphologyEx(bimg, cv2.MORPH_OPEN, kernel, iterations=1) 
    open = cv2.dilate(open, kernel)
    '''
    kernel_erode = np.ones((5,5),np.uint8)
    erosion = cv2.erode(open,kernel_erode,iterations=1)
    '''
    return open

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
        learningRate = 0.0001
        self.aimg1 = aimg
      else:
        abs_diff = np.sum(cv2.absdiff(self.aimg1, aimg))/255.0
        learningRate = self.computeLearningRate(abs_diff)
      self.bgmask = self.bgSubtractor.apply(aimg, learningRate)
      self.bgmask = cv2.compare(self.bgmask, 0, cv2.CMP_GT)
      self.bgmask = self.morph_clean(self.bgmask)
    return self.bgmask

  #===================================================
  #  turns watershed into contour
  #===================================================
  def watershedToContour(self, tempMask, markers):
    ctrList = []
    for objectCount in np.unique(markers)[2:]:

      tempMask[markers == objectCount] = 255
      tempMask[markers != objectCount] = 0
      
      tempMask.astype('int8')
      ctr, hierarchy = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      for cnt in ctr:
        ctrList.append(cnt)

    return ctrList

  #===================================================
  #  performs distance transform
  #===================================================
  def segmentation(self, fg, original, zpoint):
    fg = fg.astype('uint8')
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
    open = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel, iterations=1)
    
    sure_bg = cv2.dilate(fg, kernel)

    dist = cv2.distanceTransform(open, cv2.DIST_L2, 3)
    ret, sure_fg = cv2.threshold(dist,0.8*dist.max(),255,0)

    sure_bg = sure_bg.astype('uint8')
    sure_fg = sure_fg.astype('uint8')
    unknown = cv2.subtract(sure_bg, sure_fg)

    ret, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown==255] = 0

    originalCopy = original.copy()
    original = cv2.cvtColor(original,cv2.COLOR_GRAY2BGR)

    markers = cv2.watershed(original.astype('uint8'),markers)
    print(np.unique(markers).size-2)
    original[markers == -1] = [0,0,255]

    cntList = self.watershedToContour(originalCopy, markers)

    return cntList
    
  #===================================================
  #  performs distance transform
  #===================================================
  def zpointNormalize(self, pixelValue):
    if pixelValue < 0:
      print('negative')
      return 0
    x = int((255/(-2)*(pixelValue-2)))
    if x>255: 
      x=255
    elif x<0: 
      x=0
    return x

  #===================================================
  #  Distance score based on normalized distanceScore
  #===================================================	
  def distanceScore(self, ax, ay, bx, by):
    # dividing by the actual size of frame normalizes the value (between 0 and 1)
    dx = (ax - bx) / 160.0
    dy = (ay - by) / 60.0
    dist = math.sqrt(dx*dx + dy*dy)
    return dist
  
  #===================================================
  #  Finding x and y of centroid
  #===================================================  
  def findCenter(self, blob):
    m = cv2.moments(blob)
    if m['m00'] != 0:
      bx = int(m['m10']/m['m00'])
      by = int(m['m01']/m['m00'])
    else:
      print("illegitimate blob")
      bx = 0
      by = 0
    return bx, by
    
  #===================================================
  #  Create list of all possible matches (relationships)
  #  along with score based on similarity of shape and
  #  distance between centers
  #===================================================
  def findRelationships(self, frame1_blobs, frame2_blobs):
    relationship = []
    alpha = self.alpha
    for i in range(len(frame1_blobs)):
      for j in range(len(frame2_blobs)):
        blob_a = frame1_blobs[i]
        blob_b = frame2_blobs[j]
        # hu moment score, how similar the shape is to the other
        # lower number is better
        hm_score = cv2.matchShapes(blob_a, blob_b, 1, 0.0)
        ax, ay = self.findCenter(blob_a)
        bx, by = self.findCenter(blob_b)
        dist_score = self.distanceScore(ax, ay, bx, by)
        score = alpha*hm_score + (1-alpha)*dist_score
        relationship.append((i,j,score))
    return relationship
    
  #===================================================
  #  Given a set of potential matches (relationship),
  #  find the best match
  #===================================================
  def findBestMatch(self, relationship):
    blob_a = -1
    blob_b = -1
    minScore = 0
    for k in range(len(relationship)):
      (i, j, score) = relationship[k]
      if k == 0:
        minScore = score
        blob_a = i
        blob_b = j
      else:
        # checking if new score is lower than minScore,
        # meaning a more accurate match
        if score < minScore:
          minScore = score
          blob_a = i
          blob_b = j
    return blob_a, blob_b, minScore
    
  #===================================================
  #  Remove any relationships involving blob_a or blob_b
  #===================================================
  def removeRelationships(self, blob_a, blob_b, relationship):
    new_relationship = []
    for k in range(len(relationship)):
      (i, j, score) = relationship[k]
      if i != blob_a and j != blob_b:
        new_relationship.append((i, j, score))
    return new_relationship
    
  #===================================================
  #  Find the matches between blobs in frame 1 and 2 by:
  #
  #    1. Find all possible matches and their scores
  # 
  #    2. Find the best match and remove the matched
  #       pair of blobs from the list for further consideration
  # 
  #    3. Repeat 2 until nothing left on the list to consider
  #
  #  Function returns all the matches and the remaining
  #  unmatched relationships
  #===================================================
  def match(self, frame1_blobs, frame2_blobs):
    matches = []
    remains = self.findRelationships(frame1_blobs, frame2_blobs)
    while len(remains) != 0:
      (blob_a, blob_b, score) = self.findBestMatch(remains)
      if blob_a != -1:
        matches.append((blob_a, blob_b, score))
        remains = self.removeRelationships(blob_a, blob_b, remains)
    return matches, remains
    
  #===================================================
  #  Periodic call to publish data 
  #===================================================
  def enterOrExit(self, unmatched_tracked):
    peopleEntering = 0
    peopleExiting = 0
    
    for (i, blob) in unmatched_tracked:
     x, y = self.findCenter(blob)
     print('blob center', x, y)
     if abs(x - self.entering) <= self.error:
        peopleEntering += 1
     elif abs(x - self.exiting) <= self.error:
        peopleExiting += 1
    # print('people entering', peopleEntering, 'people exiting', peopleExiting)
    return peopleEntering, peopleExiting

  #===================================================
  #  Periodic call to refresh image and compute fg&bg and such
  #===================================================
  def periodic(self):
    dimg = self.camera['depth']
    aimg = self.camera['amp']
    zpoints = self.camera['z']

    print('periodic is running')
    
    backgroundMask = self.getBgMask(aimg)
    foreground = cv2.bitwise_and(aimg, aimg, mask = backgroundMask)
    ret,foreThresh = cv2.threshold(foreground,0,255,cv2.THRESH_BINARY)
    
    #print(foreThresh)

    if zpoints is not None:

      metersTo8bit = np.vectorize(self.zpointNormalize)
      zpoint = metersTo8bit(zpoints[:]).astype('uint8')
      ret, zpointThresh = cv2.threshold(zpoint, 10, 255, cv2.THRESH_BINARY)
      foreground = cv2.bitwise_and(zpointThresh, zpointThresh, mask = backgroundMask)
      foreground = self.morph_clean(foreground)
      cv2.imshow('foreground', self.prepare(foreground,4))

      blobs = self.segmentation(foreground, zpoint, zpoints)

      self.tracker.cartNormal = 3.0 
      self.tracker.alpha = 0.5  # dial go between shape vs dist matching   

      ximg = self.camera['x']
      yimg = self.camera['y']
      zimg = self.camera['z']
      # Pass current ximg, yimg, zimg to tracker
      self.tracker.setPointCloud(ximg, yimg, zimg)
      
      # Update tracker
      self.tracker.update(np.array(blobs))
      
      trail = (aimg/16).astype('uint8')
      trail = cv2.cvtColor(trail, cv2.COLOR_GRAY2RGB)
      trail = cv2.line(trail,(25,0),(25,60),(0,0,255),2)
      trail = cv2.line(trail,(135,0),(135,60),(0,0,255),2)
      # Updated result is in tracker.tracked, which is an array of deque
      for i in range(len(self.tracker.tracked)):
        if len(self.tracker.tracked[i]) > 0:
          # Get the deque
          q = self.tracker.tracked[i]
          x = -1
          y = -1
          
          # draw trail
          for j in q:
            if x > -1 and y > -1:
              cv2.line(trail, self.findCenter(j), (x, y), (0,0,255), 2)
            x, y = self.findCenter(j)
          
            
      cv2.imshow("trail", self.prepare(trail, 4))
    
    now = datetime.now()
    '''
    # counting the number of people in frame
    if self.countInFrame:
      tracked_blobs = len(self.tracker.matchedPairs)
      if tracked_blobs != self.peopleInFrame:
        dt, time, day, month, year, weekday = getDateTime(now)
        m1 = Message("rpi4", 16, 455, 566, "Store entrance", dt, time, day, month, year, weekday, 			      tracked_blobs, tracked_blobs)
        print("people in frame", tracked_blobs)
        self.peopleInFrame = tracked_blobs
        print(m1.dictStr())
        client.publish("topic1", json.dumps(m1.dictStr()))
    '''
        
    # tracking the number of people entering or exiting  
   
    if self.tracker.change == True:
      print('number of blobs tracked', len(self.tracker.unmatched_tracked))
      for j in self.tracker.unmatched_tracked:
        # print(self.findCenter(j))
        pass
      peopleEntered = self.tracker.currentEnter
      peopleExited = self.tracker.currentExit
      # device, deviceid, longtitude, latitude, location, time, enter, exit, people in 		building
      dt, time, day, month, year, weekday = getDateTime(now)
      m1 = Message("rpi4", 16, 455, 566, "Store entrance", dt, time, day, month, year, weekday, peopleEntered, peopleExited)
      print(m1.dictStr())
      client.publish("topic1", json.dumps(m1.dictStr()))
      print(m1.dictStr())

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
