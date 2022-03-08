#!/usr/bin/env python

import cv2
import math
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class send_it:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      raw = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = raw.copy()
    lower = np.uint8([190, 109, 109])
    upper = np.uint8([255, 255, 255])
    mask = cv2.inRange(image, lower, upper)
    image = cv2.bitwise_and(image, image, mask = mask)

    
    
    # ROI masking
    stencil = np.zeros_like(image[:,:,0])
    polygon = np.array([[0,480],[0,250], [200,50], [440,50],[640,250] , [640,480]])

    cv2.fillConvexPoly(stencil, polygon, 3)

    image = cv2.bitwise_and(image, image, mask=stencil)

    # contours
    ret, thresh = cv2.threshold(image[:,:,0], 130, 145, cv2.THRESH_BINARY)
    
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)    

    contours=[x for x in contours if len(x)> 200]

    # lines

    lines  = []

    for contour in contours :
      
      vx,vy,x,y = cv2.fitLine(contour, cv2.DIST_L1,0,0.01,0.01)


      theta = math.degrees(math.atan(vy/vx))

      if theta < 0 :
        theta = 180 + theta 

      
      if 200 < x and x < 440:
        lines.append(theta)
      

    # controls
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    
    

    vel = Twist()

    vel.angular.z = 0

    if len(lines) > 0 :
      theta = round(sum(lines)/len(lines),3)

      print(theta)

      if 30 < theta and theta < 150:
        vel.angular.z =  math.pi/2 * (90-theta)/90


    vel.linear.x = 1.95


    pub.publish(vel)
    
    cv2.drawContours(image=image, contours=contours, contourIdx=-1, color=(255,0 , 0), thickness=3, lineType=cv2.LINE_AA)


    cv2.imshow("Dashcam View", image)

    cv2.waitKey(3)

def main():
	send_it()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
    # print('hi')
    # rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		rospy.loginfo("Shutting down")
    
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
