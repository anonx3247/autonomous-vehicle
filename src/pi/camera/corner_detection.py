import cv2
import numpy as np
from .line_detection import image_to_white_points

def corner_detection(img, quality=0.99, a=100, expected_corners=4):
    dilated_mask = image_to_white_points(img)
    gray = np.float32(dilated_mask)

    dst = cv2.cornerHarris(gray,7,3,0.15)
    corners = cv2.goodFeaturesToTrack(gray, 5,quality,20)
    if corners is None:
        return False, []
    corners = np.int32(corners)
    detect_inter = False
    
    li_corners = []
    for i in corners: 
        x, y = i.ravel()
        li_corners.append((x,y))
        #print(x,y)
        cv2.circle(gray, (x, y),3,255,-1)
    if len(li_corners) >= expected_corners:
        detect_inter =True
        #print("Intersection !")
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    #gray[dst>0.1*dst.max()]=[0,0,255]

    #cv2.imshow('dst',gray)
    #if cv2.waitKey(0) & 0xff == 27:
    #   cv2.destroyAllWindows()
    #cv2.imwrite('out_test.png', img)
    print(len(li_corners))
    return detect_inter,li_corners

def detect_intersection(img, width_threshold=0.5, expected_corners=4):
    # first start by cutting the image to only get the bottom half
    height, width = img.shape[:2]
    img = img[height//2:, :]
    # then we get all white points in the image
    white_points = image_to_white_points(img)
    # we calculate a bounding box around the white points
    x, y, w, h = cv2.boundingRect(white_points)
    # we cut the image to only get the bounding box
    img = img[y:y+h, x:x+w]
    # if the width of the bounding box is more than 50% of the width of the image, we consider it as an intersection
    if w > width * width_threshold:
        return True
    return False
