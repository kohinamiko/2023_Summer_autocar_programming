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
        self.Moment = []
        rospy.spin()

    def CamCallback(self,data):

        try:
            ## Image Change
            cam_img = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convert Error!", e)

        ## Lane find --> hsv moment use
        self.LaneFind(cam_img) 

        cv2.imshow("Cam",cam_img)
        cv2.waitKey(1)
        self.Moment.clear()

    def LaneFind(self, frame):
        
        img = frame.copy()
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        height, width = img.shape[0:2] # 480 640
        
        #print(width, height)

        ## This Use first time 
        if self.Trackbar_flag == False:

            cv2.imshow("Cam",img)

            cv2.createTrackbar("LH", "Cam", 0, 180, Nothing)
            cv2.createTrackbar("UH", "Cam", 180, 180, Nothing)
            cv2.createTrackbar("LS", "Cam", 0, 255, Nothing)
            cv2.createTrackbar("US", "Cam", 20, 255, Nothing)
            cv2.createTrackbar("LV", "Cam", 211, 255, Nothing)
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

        ## 색깔따기
        # 노랑색 따기
        lower = np.array([[20,30,160]])  ## 20,30,160
        upper = np.array([[40,255,255]]) ## 40,255,255
        mask_yellow = cv2.inRange(img_hsv,lower,upper)        
        img_yellow = cv2.bitwise_and(img,img, mask = mask_yellow)

        # 하얀색 따기
        lower = np.array([[LH,LS,LV]])    ## 0,0,200
        upper = np.array([[UH,US,UV]]) ## 180,20,255
        mask_white = cv2.inRange(img_hsv,lower,upper)
        img_white = cv2.bitwise_and(img,img,mask = mask_white)


        # 흰색, 노랑색 합치기
        img_white_yellow = cv2.bitwise_or(img_white, img_yellow)
        img_gray = cv2.cvtColor(img_white_yellow, cv2.COLOR_BGR2GRAY)
        
        '''
        이미지 색깔 딸 때 원하는 색만 따서 나머지는 검은색으로 만들어서
        threshold할 때 임계값을 0으로 둬도 될 거 같음
        '''

        ## binary img
        ret, img_bin_yellow = cv2.threshold(img_yellow,thresh1,255,cv2.THRESH_BINARY)
        ret, img_bin_white = cv2.threshold(img_white,thresh1,255,cv2.THRESH_BINARY)        
        ret, img_bin = cv2.threshold(img_gray,thresh1,255,cv2.THRESH_BINARY)
        img_bin = img_bin[240:,:]
        img = img[240:,:]
        ## contours
        contour, hierarchy = cv2.findContours(img_bin,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img,contour,-1,(0,255,0),4)
        print("컨투어 개수 : ",len(contour))
        ## moment
        for c in contour :
            # 모멘트 계산
            mmt = cv2.moments(c)
            
            #print(mmt["m00"])
            #x,y 값 찾기
            cx = int(mmt["m10"]/mmt["m00"])
            cy = int(mmt["m01"]/mmt["m00"])
            self.Moment.append((cx,cy))
            cv2.circle(img, (cx,cy),5,(0,0,255),-1)

        ### 이미지 보기
        # cv2.imshow("Yellow",img_yellow)
        # cv2.imshow("White",img_white)
        cv2.imshow("White_Yellow",img_white_yellow)
        cv2.imshow("img_gray",img_gray)
        cv2.imshow("contour",img)
        # cv2.imshow("bin_yellow",img_bin_yellow)
        # cv2.imshow("bin_white",img_bin_white)
        cv2.imshow("bin_white_yellow",img_bin)
        print(self.Moment)
        


    def Shutdown(self):
        cv2.destroyAllWindows()
        print("Cam is Dead")


def Nothing(x):
    pass


if __name__ == "__main__":

    Cam = Camera()