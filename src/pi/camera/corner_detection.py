import cv2
import numpy as np


def corner_detection(img,a=100):
    mid_x = 640
    expected_corners = 3
    blur = cv2.blur(img,(6,6))
    ret,thresh1 = cv2.threshold(blur,168,255,cv2.THRESH_BINARY)
    hsv = cv2.cvtColor(thresh1, cv2.COLOR_RGB2HSV)

    # Define range of white color in HSV
    lower_white = np.array([0, 0, 168])
    upper_white = np.array([172, 111, 255])
    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel_erode = np.ones((6,6), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((4,4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    gray = np.float32(dilated_mask)

    dst = cv2.cornerHarris(gray,5,3,0.10)
    corners = cv2.goodFeaturesToTrack(gray, 5,0.5,20)
    if corners is None:
        return False, []
    corners = np.int32(corners)
    detect_inter = False
    
    li_corners = []
    for i in corners: 
        x, y = i.ravel()
        if abs(x-mid_x)<a:
            li_corners.append((x,y))
        #print(x,y)
        #cv2.circle(img, (x, y),3,255,-1)
    if len(li_corners) >= expected_corners:
        inter =True
        print("Intersection !")
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    #img[dst>0.02*dst.max()]=[0,0,255]

    #cv2.imshow('dst',img)
    #if cv2.waitKey(0) & 0xff == 27:
    #   cv2.destroyAllWindows()
    #cv2.imwrite('out_test.png', img)
    return detect_inter,li_corners
