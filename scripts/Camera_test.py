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
        #cv2.imshow("Cam",cam_img)
        cv2.waitKey(1)

    def LaneFind(self, frame):
        
        img = frame.copy()
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        img_hsl = cv2.cvtColor(img,cv2.COLOR_BGR2HLS)
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        h,l,s = cv2.split(img_hsl)

        if self.Trackbar_flag == False:

            cv2.imshow("Cam",img)

            cv2.createTrackbar("LH", "Cam", 0, 180, Nothing)
            cv2.createTrackbar("UH", "Cam", 180, 180, Nothing)
            cv2.createTrackbar("LL", "Cam", 0, 255, Nothing)
            cv2.createTrackbar("UL", "Cam", 255, 255, Nothing)
            cv2.createTrackbar("LS", "Cam", 0, 255, Nothing)
            cv2.createTrackbar("US", "Cam", 255, 255, Nothing)
            cv2.createTrackbar("thresh1", "Cam", 127, 255, Nothing)


            self.Trackbar_flag = True
        
        LH = cv2.getTrackbarPos("LH","Cam")
        LL = cv2.getTrackbarPos("LL","Cam")
        LS = cv2.getTrackbarPos("LU","Cam")

        UH = cv2.getTrackbarPos("UH","Cam")        
        UL = cv2.getTrackbarPos("UL","Cam")        
        US = cv2.getTrackbarPos("US","Cam")

        thresh1 = cv2.getTrackbarPos("thresh1","Cam")


        
        ## 색깔따기

        # 하얀색 따기
        lower = np.array([[LH,LL,LS]])
        upper = np.array([[UH,UL,US]])
        mask_white = cv2.inRange(img_hsl,lower,upper)
        img_white = cv2.bitwise_and(img,img,mask = mask_white)
        img_white_gray = cv2.cvtColor(img_white,cv2.COLOR_BGR2GRAY)
        #print(img_gray.shape)
        # mask_l = cv2.inRange(l,LS,US)
        # img_l = cv2.bitwise_and(img,img,mask=mask_l)
        # img_l_gray = cv2.cvtColor(img_l, cv2.COLOR_HLS2BGR)
        # img_l_gray = cv2.cvtColor(img_l_gray,cv2.COLOR_BGR2GRAY)
        
        ## binary img        
        ret, img_bin_white = cv2.threshold(img_white,thresh1,255,cv2.THRESH_BINARY)
        ret, img_bin_gray = cv2.threshold(img_white_gray,thresh1,255,cv2.THRESH_BINARY)
        #img_xor = cv2.bitwise_and(img_bin_white,img_bin_gray)

        ## contours
        contour, hierarchy = cv2.findContours(img_bin_gray,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img,contour,-1,(0,255,0),4)
        print("컨투어 개수 : ",len(contour)) ## 컨투어 개수가 이상하게 많이 나올 때 LV값을 높이면 사라진다.

        ## moment       
        ''' 
        점이 3개 이상 나타날 경우 카메라 기준 좌우로 나눈다.
        왼쪽 기준 x값이 가장 큰 것, 오른쪽 기준 x값이 가장 작은 것을 택함
        왠만해서 내 차로의 좌표를 구할 가능성이 높다.
        '''
        for c in contour :
            # 모멘트 계산
            mmt = cv2.moments(c)
            
            #print(mmt["m00"])
            #x,y 값 찾기
            if mmt["m00"] == 0: ## 이상한 영역이 컨투어가 나올 때가 있음
                continue

            cx = int(mmt["m10"]/mmt["m00"])
            cy = int(mmt["m01"]/mmt["m00"])
            cv2.circle(img, (cx,cy),5,(0,0,255),-1)

            # x,y좌표 저장  제어기에서 x좌표 꺼내서 왼쪽, 오른쪽 좌표 찾아낼 것
            # self.Moment[cx] = cy
            # 
            # if cx <= 320: # 카메라 마다 가운데가 다름
                # self.Left.append(cx)
            # else:
                # self.Right.append(cx)


        ### 이미지 보기        
        cv2.imshow("White",img_white)        
        cv2.imshow("white_gray",img_white_gray)        
        cv2.imshow("bin_white",img_bin_white)
        cv2.imshow("bin_white_gray",img_bin_gray)
        #cv2.imshow("xor",img_xor)
        cv2.imshow("Cam",img)
        # cv2.imshow("l",l) ## 하얀색에 강하다
        # cv2.imshow("gray",img_l_gray)
        # cv2.imshow("img_l",img_l)        
        # cv2.imshow("bin_l",img_bin_l)
        



    def Shutdown(self):
        cv2.destroyAllWindows()
        print("Cam is Dead")


def Nothing(x):
    pass


if __name__ == "__main__":

    Cam = Camera()