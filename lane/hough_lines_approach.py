
from __future__ import print_function
from dis import dis
import math

from  geometry_msgs.msg import Twist
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import matplotlib.pyplot as plt

from tf.transformations import euler_from_quaternion
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    self.bridge = CvBridge()
    self.vel = Twist()
    self.vel.linear.x = 1
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while rospy.Time.now() < now + rospy.Duration.from_sec(2):
      self.cmd_pub.publish(self.vel)
      rate.sleep()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
    


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    image = (cv_image).copy()
    
    lower = np.uint8([190, 109, 109])
    upper = np.uint8([255, 255, 255])
    mask = cv.inRange(image, lower, upper)
    image = cv.bitwise_and(image, image, mask = mask)
    # image= image[77:581,80:600]
    stencil = np.zeros_like(image[:,:,0])
    polygon = np.array([[0,480],[0,250], [200,50], [440,50],[640,250] , [640,480]])

    cv.fillConvexPoly(stencil, polygon, 1)

    image = cv.bitwise_and(image, image, mask=stencil)

    
    canny=cv.Canny(image, 50, 150)
    
   
    lines=cv.HoughLinesP(canny, rho=1, theta=np.pi/180, threshold=20, minLineLength=70, maxLineGap=70)
    

    
    thetas=[]
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            slope = (l[3]-0)/(l[2]-0)
            theta = math.degrees(math.atan(slope))
            # theta = round(theta,2)
            if theta < 0 :
                    theta = 180 + theta
            if 200<l[0] and l[0] < 440:
                thetas.append(theta)
            cv.line(image, (l[0], l[1]), (l[2], l[3]), (0,0,255), 10, cv.LINE_AA)
    else:
        print('No Lines right now!')
    self.vel.angular.z = 0
    if len(lines) > 0 :
      theta = round(sum(thetas)/len(thetas),3)
        
      print(theta)

      if 30 < theta and theta < 150:
        self.vel.angular.z = 0.9*(math.pi/2 * (90-theta)/90)
      


        
    self.vel.linear.x = 1.25


    self.cmd_pub.publish(self.vel)


    

    cv.imshow("Image window", image)
    cv.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)
        pass

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
