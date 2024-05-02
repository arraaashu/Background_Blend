import numpy as np
import cv2 #used to solve computer vision problems.
import time #used to handle various operations regarding time.
capt=cv2.VideoCapture(0)#saving the video.
cc = cv2.VideoWriter_fourcc(*'XVID')#to capture the frames.
out = cv2.VideoWriter('outputr.avi', cc, 20.0, (640, 480))
#saving the video to the output file using fourcc frames and the time using coordinates provided.

time.sleep(2)
c=0
backgrnd=0
for i in range(60):
    ret,backgrnd=capt.read()
#recording the background frame by frame for 60 secs.
backgrnd=np.flip(backgrnd,1)
# flipping the background.
while(capt.isOpened()):
    ret, image = capt.read()
    if not ret:
        break
    c += 1
    image = np.flip(image, 1)
    #hsv stands for hue, saturation, value.
    #This hsv value is used to identify the pixels in the video i.e; capt.
    #In this hsv we can change accordingly the values of the color to be blended into
    #the background.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #For masking the color we have taken initially red to be covered.
    #So, the hsv lower and higher values are to be given inorder to blend the 
    #background perfectly.
    lred = np.array([0, 120, 70])
    ured = np.array([10, 255,255])

    blend1 = cv2.inRange(hsv, lred, ured)

    lred = np.array([160, 120, 70])
    ured = np.array([180, 255, 255])

    blend2 = cv2.inRange(hsv, lred, ured)
    #concating the blends 1 and 2 using or operator '+'

    blend1 = blend1 + blend2

    b1 = cv2.morphologyEx(blend1, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    b1 = cv2.morphologyEx(blend1, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))
    #creating an inverted blend to segment out the red color from the frames.
    blend2 = cv2.bitwise_not(blend1)
    #segment the red color part out of the frame using bitwise and with the inverted blend.
    res2 = cv2.bitwise_and(image, image, mask=blend2)

    #creating image showing static background frame pixels only for the blended region
    res1 = cv2.bitwise_and(backgrnd, backgrnd, mask=blend1)

    finalOutput = cv2.addWeighted(res1, 1, res2, 1, 0)
    cv2.imshow("A beautiful Background Blend", finalOutput)
    out.write(finalOutput)
    k=cv2.waitKey(10)
    if k==27:
        break

capt.release()
out.release()
cv2.destroyAllWindows()