import numpy as np
import cv2
import math,sys     
import serial as ser
import time
import struct
##################################################################################################################################################
global Ki, Kd, Kp, last_time, integrat, prev_err
Ki=1
Kd=1
Kp=1
integrat=0
last_time=0
prev_err=0

def nothing(x):
    pass
##################################################################################################################################################

def getThresoldValue(name):
    cv2.namedWindow(name)
    cv2.createTrackbar('hMax', name, 255, 255,nothing)
    cv2.createTrackbar('hMin', name, 0, 255,nothing)
    cv2.createTrackbar('sMax', name, 255, 255,nothing)
    cv2.createTrackbar('sMin', name, 0, 255,nothing)
    cv2.createTrackbar('vMax', name, 255, 255,nothing)
    cv2.createTrackbar('vMin', name, 0, 255,nothing)
    
    cap = cv2.VideoCapture(-1)
    
    while (True):
        ret,frame = cap.read()
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        hMin = cv2.getTrackbarPos('hMin',name)
        sMin = cv2.getTrackbarPos('sMin',name)
        vMin = cv2.getTrackbarPos('vMin',name)
        hMax = cv2.getTrackbarPos('hMax',name)
        sMax = cv2.getTrackbarPos('sMax',name)
        vMax = cv2.getTrackbarPos('vMax',name)

        mask = cv2.inRange(hsv,(hMin,sMin,vMin),(hMax,sMax,vMax))
        cv2.imshow(name,mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            print ("Success")
            break

    minValues = np.array([hMin,sMin,vMin])
    MaxValues = np.array([hMax,sMax,vMax])

    return (minValues,MaxValues,mask)


##################################################################################################################################################

def locate(mask):
    cntSet = detectContours(mask)
    resList = []

    '''for cnt in cntSet:
        centroid = findCentroid(cnt)
        resList.append(centroid)'''
    resList = findCentroid(cntSet)
    #print resList
    return resList
            
##################################################################################################################################################

def dist(ori,res):
    dist = [-(res[0]-ori[0]),-(res[1]-ori[1])]
    return (dist)

##################################################################################################################################################

def locateTC():
    cap = cv2.VideoCapture(1)
    ret,frame = cap.read()
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(hsv,tcMin,tcMax)

    tcCenter = findCentroid(mask)
    return tcCenter

##################################################################################################################################################

def detectContours(mask):
    minCntArea = 0
    cap = cv2.VideoCapture(1)
    ret,frame = cap.read()
    
    #hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    #mask = cv2.inRange(hsv,objMin,objMax)

    (mask,cntSet, _) = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print len(cntSet)
    cv2.drawContours(mask,cntSet,-1,(255,255,255), 1)
    cv2.imshow('mask',mask)

    if cv2.waitKey(0) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
    c = max(cntSet,key = cv2.contourArea)
    '''for cnt in cntSet:
        if (cv2.contourArea(cnt)<minCntArea):
            cntSet = np.delete(cntSet,cnt)
            minCntArea = cv2.contourArea(cnt)'''

    return c

##################################################################################################################################################

def findCentroid(cnt):
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        #if centroid found
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        #assuming no centroid
        cx = -1
        cy = -1
    return (np.array([cx,cy]))


####################################################################################################################################################################

#global cMin, cMax
#cMin , cMax, _ = getThresoldValue('origin')
#print cMin,cMax
#origin=locate(cMin,cMax)
origin = [319,478]
#global tcMin, tcMax
#tcMin , tcMax, _ = getThresoldValue('green')
#print tcMin,tcMax
global green
#green=locate(tcMin,tcMax)
green = [319,239]
resMin , resMax, mask = getThresoldValue('red')
#print resMin,resMax
red = locate( mask)
green = dist(origin,green)
red = dist(origin,red)
print green,red


ratio = [138.0/638.0,108.0/478.0]

green = [0,21]
red[0] = red[0]*ratio[0]
red[1] = red[1]*ratio[1]
print green,red,"f"
depth = 53
ox=90
oy=0
x = ox+math.atan(red[0]/depth)*180.0/3.14
y = oy+math.atan(red[1]/depth)*180.0/3.14
print x,y
arduino = ser.Serial('/dev/ttyACM0',9600)
time.sleep(2)
arduino.write(struct.pack('>BB',x,y))