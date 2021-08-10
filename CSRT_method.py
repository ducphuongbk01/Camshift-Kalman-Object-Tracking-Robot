import cv2 

def drawBox(img, box):
    x,y,w,h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    cv2.rectangle(img, (x,y), ((x+w),(y+h)),(255,0,255),3,1)
    cv2.putText(img, "Tracking!",(75,75),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,0.7,(0,255,0),2)

cap = cv2.VideoCapture(0)

tracker = cv2.TrackerCSRT_create()
ret, frame = cap.read()
bbox = cv2.selectROI("Tracking",frame,False)
tracker.init(frame,bbox)

while True:
    
    #Show how many frames per second
    timer = cv2.getTickCount()
    ret, frame = cap.read()
    fps = cv2.getTickFrequency()/(cv2.getTickCount()- timer)
    cv2.putText(frame, str(int(fps)),(75,50),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,0.7,(0,0,255),2)
    
    success, bbox = tracker.update(frame)
    print(bbox)

    if success:
        drawBox(frame, bbox)
    else:
        cv2.putText(frame, "Cannot find the Object!",(75,75),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,0.7,(255,0,0),2)

    
    if ret:
        cv2.imshow("Camera",frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break



cap.release()
cv2.destroyAllWindows()

