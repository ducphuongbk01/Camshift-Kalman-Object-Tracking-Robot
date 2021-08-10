#import the necessary packages
import numpy as np
import cv2
import Kalman_Filter 
import time
import serial
from threading import Thread
#initialize the current frame of the video, along with the list of ROI point
#ROI point along with whether or not this is input mode
frame = None
roiPts = []
inputMode = False
error = 0
ser = None

try:
    ser = serial.Serial('/dev/serial0', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)
except:
    print('Cannot connect to serial Port!!!')

def writeInfo(serial, error):
    try:
        txbuff = []
        if error >= 0:
            sign = '+'
        else:
            sign = '-'
            error = - error
        error = str(error)
        error = str.encode(error)
        error = list(error)
        txbuff.append(0x02)
        txbuff.append(sign)
        if(len(error)<3):
            error.insert(0,0x30)
        for i in range(3):
            txbuff.append(error[i])
        txbuff.append(0x03)
        print(txbuff)
        serial.write(txbuff)
        print('Successfully Transmitted !!!')
    except:
        print('Cannot Transmitted !!!')

def selectROI(event, x, y, flags, param):
    #grab the reference to the current frame, list of ROI point
    #and whether or not it is ROI selection mode
    global frame, roiPts, inputMode

    #if we are in ROI selection mode, the mouse wa clicked,
    #and we do not already have 4 points
    #the update the list of ROI points with the (x, y) location of the click
    #and draw the circle
    if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts)<4:
        roiPts.append((x, y))
        cv2.circle(frame, (x, y),4,(0,255,0),2)
        cv2.imshow("Frame", frame)

def removeNoisebyThresholding(img):
    #kernel = np.ones((3,3),np.uint8)
    #result = cv2.GaussianBlur(img, (1,1), 0)
    result = img
    cv2.threshold(result,100,255,0,result)
    cv2.imshow("BackProj_Thresh",result)
    return result

#Fnc to sort 4 points in desire order
def reorder(myPoints):
    myPointsNew = np.zeros_like(myPoints)
    myPoints = myPoints.reshape((4,2))
    add = myPoints.sum(1)  #Sum of row --> return row vector
    myPointsNew[0] = myPoints[np.argmin(add)] #find a point that return the minimum of the sum of row
    myPointsNew[3] = myPoints[np.argmax(add)]
    diff = np.diff(myPoints,axis=1)   #Subtraction of row --> return column vector
    myPointsNew[1] = myPoints[np.argmin(diff)] #find a point that return the minimum of the subtraction of row
    myPointsNew[2] = myPoints[np.argmax(diff)]
    return myPointsNew

def main():
    # grab the reference to the current frame 
    # list of ROI points and whether or not it is ROI selection mode
    global frame, roiPts, inputMode

    cap = cv2.VideoCapture(0)

    #setup the mouse call back
    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", selectROI)

    #initialize the termination criteria for camshift
    #indicating a maximum of ten iterations or movement by a least one pixel
    #along with the bounding box of the ROI
    termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
    roiBox = None
    pTime = 0


    #keep looping over the frames
    while True:
        #grab the current frames
        ret, frame = cap.read()

        #check to see if we have reached the end of the video
        if not ret:
            continue

        #if the see if the ROI has been computed
        if roiBox is not None:
            #convert the current frame to the HSV color space
            #and perform mean Shift
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.imshow("HSV",hsv)
            backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)
            cv2.imshow("BackProj",backProj)
            backProj = removeNoisebyThresholding(backProj)

            #apply CamShift to the back projection, 
            #convert the points to a bounding box
            #and draw them
            r, roiBox = cv2.CamShift(backProj, roiBox, termination)
            #print(type(r))
            
            r = kalman.compute_Para(r)
            a = int(r[0][0])
            b = int(r[0][1])
            cv2.circle(frame, (a,b),3,(0,255,0),3)
            pts = np.int0(cv2.boxPoints(r))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            
            # Draw line and the center of box
            cv2.line(img = frame, pt1 = (320, 0), pt2 = (320,480), color = (0, 0, 255),thickness = 2)
            cv2.line(img = frame, pt1 = (0, 240), pt2 = (640,240), color = (0, 0, 255),thickness = 2)
            cv2.circle(frame, (320,240),3,(0,0,255),3)
            cv2.line(img = frame, pt1 = (a, b), pt2 = (a,240), color = (255, 0, 0),thickness = 2)
            cv2.line(img = frame, pt1 = (a, b), pt2 = (320,b), color = (255, 0, 0),thickness = 2)

            #Compute the error and transmitt the Info to micro controller
            error = 320 - a
            print(error)
            # t = Thread(target = writeInfo, args = (ser, error))
            # t.deamon = True
            # t.start()
            writeInfo(ser, error)

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        cv2.putText(frame, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)
        #show the frame and recoed if the user presses a key
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xff

        #handle if the 'i' key pressed, then go into ROI selection mode
        if key == ord("i") and len(roiPts) <4:
            #indicate that we are in input mode and clone the frame
            inputMode = True
            orig = frame.copy()

            #keep looping until 4 reference ROI points have been selected
            #press any key to exit ROI selection mode once 4 point have been selected
            while len(roiPts) < 4:
                cv2.imshow("Frame", frame)
                cv2.waitKey(0)

            #determine the top-left and bottom-right points
            roiPts = np.array(roiPts)
            roiPts = reorder(roiPts)

            kalman_initialPara = (((roiPts[1][0]-roiPts[0][0])//2,(roiPts[2][1]-roiPts[0][1])//2),((roiPts[1][0]-roiPts[0][0]),(roiPts[2][1]-roiPts[0][1])),0)
            kalman = Kalman_Filter.KalmanFilter(initial_r=kalman_initialPara)
            
            s = roiPts.sum(axis=1)
            tl = roiPts[np.argmin(s)]
            br = roiPts[np.argmax(s)]

            #grab the ROI for the bounding box 
            #and convert it to the HSV color space
            roi = orig[tl[1]:br[1], tl[0]:br[0]]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            #compute a HSV histogram for the ROI and store the bounding box
            roiHist = cv2.calcHist([roi],[0],None,[64],[0,180])
            roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
            roiBox = (tl[0], tl[1], br[0]-tl[0], br[1]-tl[1])
        #if the 'q' key is pressed, stop the loop
        elif key == ord("q"):
            break
    
    #clean up the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
