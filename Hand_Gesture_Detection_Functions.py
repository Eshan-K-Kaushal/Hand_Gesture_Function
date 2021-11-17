import cv2
import math
import numpy as np
import pyautogui as p
import time as t

# camera connection
def nothing(x):
    pass
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) #0 for my stock webcam
#lower_h between 90 - 96
#lower_s between 40 to 50
#lower_v near 60

# but in my case, the values were
#0-25-0 for H-S-V
# making the sliding bars
cv2.namedWindow("Color Tuning", cv2.WINDOW_NORMAL) #creating a window
cv2.resizeWindow("Color Tuning", (350,350)) #tnhe size of the window
cv2.createTrackbar("Threshold", "Color Tuning", 0, 255, nothing) #no function being called

#color detection track

# format = ('<Parameter>', '<Window name in which it has to be displayed>',\
# '<start value for the slider>', '<count/range>', \
# '<what function on call for change>')

#find about trackbars
cv2.createTrackbar("Lower-H", "Color Tuning", 0, 255, nothing)
cv2.createTrackbar("Lower-S", "Color Tuning", 0, 255, nothing)
cv2.createTrackbar("Lower-V", "Color Tuning", 0, 255, nothing)
cv2.createTrackbar("Upper-H", "Color Tuning",255, 255, nothing)
cv2.createTrackbar("Upper-S", "Color Tuning", 255, 255, nothing)
cv2.createTrackbar("Upper-V", "Color Tuning", 255, 255, nothing)

while True:
    _, frame = cap.read() #reading the captured stuff, igonre all other
    frame = cv2.flip(frame, 2) #to show the real image unlike a mirror
    frame = cv2.resize(frame, (650,650))
    #making a lil window to get the hand configs
    cv2.rectangle(frame, (0,1), (500,800), (255, 0, 0), 0)
    crop_img = frame[1:800, 0:500] #from x to x'

    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    # we store the hand part - only - in the hsv for further analysis

    # here we track the hand on the color basis
    low_h = cv2.getTrackbarPos("Lower-H", "Color Tuning")
    low_s = cv2.getTrackbarPos("Lower-S", "Color Tuning")
    low_v = cv2.getTrackbarPos("Lower-V", "Color Tuning")

    up_h = cv2.getTrackbarPos("Upper-H", "Color Tuning")
    up_s = cv2.getTrackbarPos("Upper-S", "Color Tuning")
    up_v = cv2.getTrackbarPos("Upper-V", "Color Tuning")

    #storing the sloder vals to be able to get the hand according to our own
    #style
    lower_bound = np.array([low_h, low_s, low_v]) #1D array
    upper_bound = np.array([up_h, up_s, up_v])

    # creating mask on the basis of the color filters
    #creating the mask
    mask = cv2.inRange(hsv,lower_bound, upper_bound)
    #filter to go with the mask with the image
    filterr = cv2.bitwise_and(crop_img, crop_img, mask = mask)

    #now for complexity defects for the angles between the fingers

    # contours need to be white or highlighted
    # on the other hand, the the background should be black so that
    # we can see stuff clearly
    mask1 = cv2.bitwise_not(mask) #this does the black and white swap
    mg = cv2.getTrackbarPos("Threshold", "Color Tuning") #getting trackbar's vals
    ret, thresh = cv2.threshold(mask1, mg, 255, cv2.THRESH_BINARY)
    #threshbinary makes it single channel or makes it better the existing ones
    # this is the threshold of the pixels we want to see or ignore
    #not using it here since we used the threshold or the color tuning to detect the stuff
    dilate = cv2.dilate(thresh, (3,3), iterations = 6)
    # dilate handles the noise
    # or we can also perform EROSION for more complex scenarios

    #finding contours here
    #and then we use the contours to find the convexity hull and convexity defects
    # we bind the hull and the defect to the gestures to various functions later
    cnts, hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #making the finding for defects and hulls now

    try:
        #print("Inside Try")
        #if no contour, then an error might come up and hence we used try
        # research on this

        cm = max(cnts, key=lambda x: cv2.contourArea(x))
        #find the contour with the max area
        epsilon = 0.0005*cv2.arcLength(cm, True)
        #figure being made is better
        data = cv2.approxPolyDP(cm, epsilon, True)
        # to enhance the distortions by the epsilon
        hull = cv2.convexHull(cm)

        cv2.drawContours(crop_img, [cm], -1, (50,50,150), 2)
        cv2.drawContours(crop_img, [hull], -1, (0,255,0), 2)

        # find the convexity defects
        #convexity defects refers to the defects or the bends in the line
        #now this bend can be for any angle
        #we only take the ones that are less than 90 degs
        #since the angle between any two fingers can't be more than 90 degs
        hull = cv2.convexHull(cm, returnPoints = False)
        defects = cv2.convexityDefects(cm, hull)
        count_defects = 0
        #print("toke")
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0] #get only the first points for all the rows

            start = tuple(cm[s][0])
            end = tuple(cm[e][0])
            far = tuple(cm[f][0])

            #find the length of all the sides of the triangle

            a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

            #cosine rule here

            angle = (math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 180)/3.14

            if angle <=60:
                count_defects += 1
                cv2.circle(crop_img, far, 5, [255,255,255], -1)
                #find the arguments meaning

        if count_defects ==0:
            cv2.putText(frame, "", (50,50), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0,0,255), 1)
        elif count_defects == 1:
            p.press("space")
            cv2.putText(frame, "Play/Pause", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0, 0, 255), 1)
        elif count_defects == 2:
            p.press("up")
            cv2.putText(frame, "Volume Up", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0, 0, 255), 1)
        elif count_defects == 3:
            p.press("down")
            cv2.putText(frame, "Volume Down", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0, 0, 255), 1)
        elif count_defects == 4:
            p.press("right")
            cv2.putText(frame, "Fast Fwd", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0, 0, 255), 1)
        #if count_defects ==5:
            #cv2.putText(frame, "", (50,50), cv2.FONT_HERSHEY_SIMPLEX,
                        #2, (0,0,255), 1)
        else:
            pass
    except:
        pass
    cv2.imshow("Threshold", thresh)
    cv2.imshow('filter==', filterr)
    cv2.imshow("Result", frame)

    key = cv2.waitKey(25) &0xFF
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()





