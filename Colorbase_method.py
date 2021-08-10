import cv2
import numpy as np
  
camera = cv2.VideoCapture(0)
kernel = np.ones((5,5), dtype="uint8")
colorSet = False
  
def selectColor(event, x, y, flags, param):
    global colorLower, colorUpper, colorSet
  
    if event == cv2.EVENT_LBUTTONDOWN:
        color = hsv[y,x]
        colorLower = np.array([color[0]-10, color[1]-20, color[2]-20])
        colorUpper = np.array([color[0]+10, 255, 255])
        colorSet = True
        print(colorLower, colorUpper)
  
cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", selectColor)
  
while True:
    global hsv
  
    grabbed, frame = camera.read()
  
    if not grabbed:
        continue
  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (3,3), 0)
  
    if colorSet:
        mask = cv2.inRange(hsv, colorLower, colorUpper)
  
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
  
        cnts,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
        if len(cnts) > 0:
            cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
  
            rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
            cv2.drawContours(frame, [rect], -1, (0,255,0), 2)
  
            cv2.imshow("Binary", mask)
  
    cv2.imshow("Tracking", frame)
  
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
  
camera.release()
cv2.destroyAllWindows()