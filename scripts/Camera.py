#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import cv2
import numpy as np
import rospy


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class Camera():

    def __init__(self):
        
        ## 로스 변수 선언
        rospy.init_node("Camera")
        rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,self.CamCallback)
        rospy.on_shutdown(self.Shutdown)

        ## 일반 변수 선언
        self.cv_bridge = CvBridge()   
        self.Trackbar_flag = False 

        rospy.spin()

    def CamCallback(self,data):

        try:
            cam_img = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convert Error!", e)

        self.LaneFind(cam_img)
        cv2.imshow("Cam",cam_img)
        cv2.waitKey(1)

    def LaneFind(self, frame):
        
        if self.Trackbar_flag == False:

            cv2.imshow("Cam",frame)

            cv2.createTrackbar("LH", "Cam", 0, 180, Nothing)
            cv2.createTrackbar("UH", "Cam", 180, 180, Nothing)
            cv2.createTrackbar("LS", "Cam", 0, 255, Nothing)
            cv2.createTrackbar("US", "Cam", 255, 255, Nothing)
            cv2.createTrackbar("LV", "Cam", 0, 255, Nothing)
            cv2.createTrackbar("UV", "Cam", 255, 255, Nothing)
            cv2.createTrackbar("thresh1", "Cam", 127, 255, Nothing)

            self.Trackbar_flag = True
        
        LH = cv2.getTrackbarPos("LH","Cam")
        LS = cv2.getTrackbarPos("LS","Cam")
        LV = cv2.getTrackbarPos("LV","Cam")

        UH = cv2.getTrackbarPos("UH","Cam")        
        US = cv2.getTrackbarPos("US","Cam")        
        UV = cv2.getTrackbarPos("UV","Cam")

        thresh1 = cv2.getTrackbarPos("thresh1","Cam")


        img = frame.copy()
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        ## 색깔따기
        # 노랑색 따기
        lower = np.array([[20,30,160]])
        upper = np.array([[40,255,255]])
        mask_yellow = cv2.inRange(img_hsv,lower,upper)        
        img_yellow = cv2.bitwise_and(img,img, mask = mask_yellow)

        # 하얀색 따기
        lower = np.array([[0,0,200]])
        upper = np.array([[180,20,255]])
        mask_white = cv2.inRange(img_hsv,lower,upper)
        img_white = cv2.bitwise_and(img,img,mask = mask_white)


        # 흰색, 노랑색 합치기
        img_white_yellow = cv2.bitwise_or(img_white, img_yellow)
                
        ## binary img
        ret, img_bin_yellow = cv2.threshold(img_yellow,thresh1,255,cv2.THRESH_BINARY)
        ret, img_bin_white = cv2.threshold(img_white,thresh1,255,cv2.THRESH_BINARY)
        ret, img_bin_white_wellow = cv2.threshold(img_white_yellow,thresh1,255,cv2.THRESH_BINARY)



        ### 이미지 보기
        cv2.imshow("Yellow",img_yellow)
        cv2.imshow("White",img_white)
        cv2.imshow("White_Yellow",img_white_yellow)
        cv2.imshow("hsv",img_hsv)

        cv2.imshow("bin_yellow",img_bin_yellow)
        cv2.imshow("bin_white",img_bin_white)
        cv2.imshow("bin_white_yellow",img_bin_white_wellow)
        


    def Shutdown(self):
        cv2.destroyAllWindows()
        print("Cam is Dead")


def Nothing(x):
    pass


if __name__ == "__main__":

    Cam = Camera()