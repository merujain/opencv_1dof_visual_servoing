#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

bridge = CvBridge()

def nothing(x):
  pass

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def show_image2(img):
  cv2.imshow("Image2 Window", img)
  cv2.waitKey(3)

res = 0
mask = 0
cv_image=0
cv_image1=0
cv_image2=0
iter1=0
iter2=0
lap1=[0,0]
lap2=[0,0]
lap_total=0
dxc=0
dyx=0
point_param = np.array([[0,0],[0,0]])
pub1=None
pub2 = None
current_position=0
goal_position =0
data_plot = np.array([[0,0,0,0,0]])
vel1=0
vel=0
def image_callback(img_msg):

  # print("Sub")
  global bridge
  global lap1,lap2,res,cv_image1,cv_image2,iter1
  try:
      cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
      cv_image = cv2.resize(cv_image,(1200,720))
      M = cv2.getRotationMatrix2D((600,360), 180, 1)
      cv_image = cv2.warpAffine(cv_image, M, (1200, 720)) 
      
  except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

  
  # for green
  hsv = np.uint8(cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV))
  l_h = 56
  l_s = 76
  l_v = 39
  u_h = 73
  u_s = 255
  u_v = 255
  lower = np.array([l_h,l_s,l_v])
  upper = np.array([u_h,u_s,u_v])  
  mask = cv2.inRange(hsv,lower,upper)
  res =  cv2.bitwise_and(np.uint8(cv_image),np.uint8(cv_image),mask=np.uint8(mask))
  lap1 = cv2.GaussianBlur(res,(5,5),0)
  lap1[:,:,0] = 0
 
  lap1[:,:,2] = 0
  # lap1 = np.uint8(np.absolute(cv2.Laplacian(res,cv2.CV_64F)))

  
  # for blue
  hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
  l_h = 0
  l_s = 96
  l_v = 58
  u_h = 53
  u_s = 250
  u_v = 255
  lower = np.array([l_h,l_s,l_v])
  upper = np.array([u_h,u_s,u_v])  
  mask = cv2.inRange(hsv,lower,upper)
  res =  cv2.bitwise_and(cv_image,cv_image,mask=mask)
  lap2 = cv2.GaussianBlur(res,(5,5),0)
  lap2[:,:,0] = 0
  lap2[:,:,1] = 0
  
  # lap2 = np.uint8(np.absolute(cv2.Laplacian(res,cv2.CV_64F)))


  # print(lower,upper)

  if iter1==0:
    cv_image1 = cv_image
    cv_image2 = cv_image
    iter1=1
  else:
    # print("Y0")
    cv_image2=cv_image1
    cv_image1=cv_image
    


def createWindowsAndTrackbars():     
  # print("create")
  cv2.namedWindow("Tracking1")
  cv2.createTrackbar("LH","Tracking1",82,255,nothing)
  cv2.createTrackbar("LS","Tracking1",62,255,nothing)
  cv2.createTrackbar("LV","Tracking1",61,255,nothing)

  cv2.createTrackbar("UH","Tracking1",89,255,nothing)
  cv2.createTrackbar("US","Tracking1",255,255,nothing)
  cv2.createTrackbar("UV","Tracking1",255,255,nothing)
  

def intersection((x1,x2,y1,y2),(a1,a2,b1,b2)):
  ans = 0 
  if  (x1<a1<x2 and y1<b1<y2) or (x1<a2<x2 and y1<b1<y2) or (x1<a1<x2 and y1<b2<y2) or (x1<a2<x2 and y1<b2<y2):
    ans=1

  if a1<x1<a2 and b1<y1<b2 and a1<x2<a2 and b1<y2<b2:
    ans=1

  return ans 

# def black_percent(p):
#   global lap_total
#   (X1,X2,Y1,Y2) = p
#   gr = np.uint8(cv2.cvtColor(lap_total.copy(),cv2.COLOR_BGR2GRAY))
#   num = cv2.countNonZero(gr[Y1:Y2,X1:X2])
#   print(100*num/((Y2-Y1)*(X2-X1)))

(px1,py1,px2,py2) = (0,0,0,0)
iter3=0
def do():
    global res,mask,cv_image1,cv_image2,lap1,point_param,iter2,lap2
    global dxc,dyc,px1,py1,px2,py2,iter3,lap_total


    if len(lap1)>2 and len(lap2)>2:

      # Find the total weighted image of blue and green
      lap_total = cv2.addWeighted(lap1,0.5,lap2,0.5,0)

      # find the contours in green and blue
      #green
      lap_copy=lap1.copy()
      lap_copy = np.uint8(lap_copy)
      blur = cv2.GaussianBlur(lap_copy,(5,5),0)
      blur = cv2.GaussianBlur(blur,(5,5),0)
      gray = np.uint8(cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY))
      gray_copy=gray.copy()
      _,contours1,_ = cv2.findContours(gray_copy,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #blue
      lap_copy=lap2.copy()
      lap_copy = np.uint8(lap_copy)
      blur = cv2.GaussianBlur(lap_copy,(5,5),0)
      blur = cv2.GaussianBlur(blur,(5,5),0)
      gray = np.uint8(cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY))
      gray_copy=gray.copy()
      _,contours2,_ = cv2.findContours(gray_copy,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

      #calculate biggest contour
      ar=0
      C=0
      n =0
      for cnt in contours1:
        if ar<cv2.contourArea(cnt) and cv2.contourArea(cnt)>200 :
          ar = cv2.contourArea(cnt)
          C=cnt
          n=1
      for cnt in contours2:
        if ar<cv2.contourArea(cnt) and cv2.contourArea(cnt)>200 :
          ar = cv2.contourArea(cnt)
          C=cnt
          n=2

      # find the contours intersection of blue and green
      pt = np.array([[0,0,0,0]])
      num=0 
      if n==1:
        con = contours2
      else:
        con = contours1
      (x11,y11,w11,h11) = cv2.boundingRect(C)
      p1 = (x11,x11+w11,y11,y11+h11)
      cv2.rectangle(lap_total, (x11,y11),(x11+w11,y11+h11),(0,255,255),2)
      pt = np.append(pt,np.array([[x11,x11+w11,y11,y11+w11]]),axis=0)
      num=num+1
      for j in contours2:
        (x22,y22,w22,h22) = cv2.boundingRect(j)
        p2 = (x22,x22+w22,y22,y22+h22)
        if intersection(p1,p2)==1 and cv2.contourArea(j)>200:
          cv2.rectangle(lap_total, (x22,y22),(x22+w22,y22+h22),(0,255,255),2)
          pt = np.append(pt,np.array([[x22,x22+w22,y22,y22+w22]]),axis=0)
          num=num+1
      pt = np.delete(pt,(0),axis=0)
      

      # Find the center of all the rectangle combined and use it as the tracking point
      x = 0
      y = 0
      # for i in range(0):
      #   x = x + (pt[i,0] + pt[i,1])/2
      #   y = y + (pt[i,2] + pt[i,3])/2 
      # x = x/2
      # y = y/2
      x = (pt[0,0] + pt[0,1])/2
      y = (pt[0,2] + pt[0,3])/2 

      # Find the difference in desired and current position
      if  0<x<500 or 700<x<1200:
        dxc = x-600
      else:
        dxc=0
    # else:
    #   dxc=0
   
    # cv2.rectangle(lap1, (450,0),(750,720),(0,0,255),2)

    # # cv2.circle(lap,(xc,yc),5,(255,255,0),-1)
    # cv2.circle(lap1,(600,360),5,(255,255,0),-1)

      cv2.line(lap1,(600,y),(600+dxc,y),(0,255,255),2)
      cv2.circle(lap_total,(x,y),5,(255,255,255),-1) 
      cv2.rectangle(lap_total, (450,0),(750,720),(0,255,255),2)
      cv2.imshow("Image Window5", lap_total)
      cv2.waitKey(30)


def motor_callback(msg):
  global data_plot,current_position,vel1,vel
  current_position = msg.current_pos
  goal_position  = msg.goal_pos
  time = msg.header.seq
  vel1 = msg.velocity
  data_plot = np.append(data_plot,np.array([[time,current_position,goal_position,vel1,vel]]),axis=0)
  # print(time)

  # print(current_position,goal_position)


def motor():
  global dxc,dyc,current_position,pub1,pub2,data_plot,vel1,vel
  lst =0.5/640
  const = (1.0/100)*(0.5/0.0038)
  pos = current_position
  p_gain1 = 0.5
  if 1<current_position<5.2:
    error = dxc*lst
    des_pos = current_position + p_gain1*error
    vel = const*error
    print(vel," ",vel1)
    pub1.publish(des_pos)
    pub2.publish(vel)
  else :
    pub1.publish(current_position)
    pub2.publish(0)
    yo=1

  if dxc==0 and data_plot[-1,0] - data_plot[1,0]>200:
    data_plot = np.delete(data_plot,(0),axis=0)


    plt.plot(data_plot[:,0]- data_plot[0,0],(data_plot[:,1]),'r',label = 'current pos')
    plt.plot(data_plot[:,0]- data_plot[0,0],(data_plot[:,2]),'g',label = 'desired pos')
    plt.plot(data_plot[:,0]- data_plot[0,0],(data_plot[:,3]),'b',label = 'current vel')
    plt.plot(data_plot[:,0]- data_plot[0,0],(data_plot[:,4]),'y',label = 'pub vel')

    plt.legend()
    plt.show()
    data_plot = np.array([[0,0,0,0,0]])

p=0
def main():
  global pub1,pub2,dxc,data_plot,p


  rospy.init_node('opencv_example', anonymous=True)
  # createWindowsAndTrackbars()
  sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

  if p==0:
    rate = rospy.Rate(5)
    rate.sleep()
    p=1
  sub_motor = rospy.Subscriber("/tilt_controller/state", JointState, motor_callback)
  pub1 = rospy.Publisher("/tilt_controller/command",Float64,queue_size=1)
  pub2 = rospy.Publisher("/pan_controller/command",Float64,queue_size=1)
  
  while not rospy.is_shutdown():
    do()
    motor()
  
  rospy.spin()

if __name__ == '__main__':
  main()
