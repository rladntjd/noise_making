import cv2 as cv
from math import sin, cos, sqrt, pi
import numpy as np

img_y_start = 0
img_x_end = 0
img_y_end = 0
img_x_start = 0

def mouse_callback(event, x, y, flags, param): 
    global img_y_start, img_x_end, img_y_end, img_x_start
    if event == cv.EVENT_LBUTTONDOWN: 
        drawing = True
        img_x_start, img_y_start = x, y 
        print("start position : %d %d", img_x_start, img_y_start)
    # elif event == cv2.EVENT_MOUSEMOVE: 
        # if drawing: 
        #     cv2.rectangle(frame, (xi, yi), (x, y), (B[0], G[0], R[0]), -1) 
    elif event == cv.EVENT_LBUTTONUP: 
        drawing = False 
        img_x_end, img_y_end = x, y 
        print("end position : %d %d", img_x_end, img_y_end)

if __name__ == '__main__':
    image_raw = cv.imread("depth.png", cv.IMREAD_UNCHANGED)
    image = np.zeros((480, 640), np.uint16)
    image = np.array(image_raw)

    cv.namedWindow("Depth window", 1)
    cv.setMouseCallback("Depth window", mouse_callback)
    while(True):

        cv.imshow("Depth window", image)
        key = cv.waitKey(1)
        if key == 27:
            break

    cropped_img = image[img_y_start: img_y_end, img_x_start: img_x_end]
    array = [[0]*2 ]*1
    for i in range (img_y_end - img_y_start):
        for j in range (img_x_end - img_x_start):
            array_len = len(array)
            found = 0
            # for arr_index in range (array_len):
            for k in range (array_len):
                if cropped_img[i][j] == array[k][0]:
                    array[k][1] += 1
                    found = 1

            if found == 0: 
                array.append([cropped_img[i][j], 1])
                array.sort(key=lambda x : x[0])
    print(array)

    sum = 0
    for i in range (len(array)) :
        sum += array[i][1]
    print((img_y_end - img_y_start) * (img_x_end - img_x_start))
    print(sum)

