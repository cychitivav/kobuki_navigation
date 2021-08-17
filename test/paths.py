#!/usr/bin/python
import numpy as np
import cv2
from matplotlib import pyplot as plt


def get_skeleton(image):
    _, th = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)        

    kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (3,3)) 
    op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

    skel = cv2.ximgproc.thinning(op)
    
    return skel


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[0:2]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1)

    vertical = cv2.warpAffine(image, rot_mat, image.shape[0:2], flags=cv2.INTER_CUBIC)
    im = vertical.copy()

    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if i < 100 or j < 100 or j > 924 or i > 924:
                im[i,j] = 205
            else:                    
                neighbor = 0
                if vertical[i+1,j] < 43.0:
                    neighbor += 1
                if vertical[i-1,j] < 43.0:
                    neighbor += 1
                if vertical[i+1,j-1] < 43.0:
                    neighbor += 1
                if vertical[i+1,j+1] < 43.0:
                    neighbor += 1
                if vertical[i-1,j+1] < 43.0:
                    neighbor += 1
                if vertical[i-1,j-1] < 43.0:
                    neighbor += 1
                if vertical[i,j+1] < 43.0:
                    neighbor += 1
                if vertical[i,j-1] < 43.0:
                    neighbor += 1
                    
                if neighbor >= 5:
                    im[i,j] = 0.0  
    
    return im

    

if __name__ == "__main__":
    
    
    image = cv2.imread('map/map.pgm', 0)
    rotated = rotate_image(image, -7.66)
    skel = get_skeleton(rotated)
    
    alpha = 0.3
    costmap = skel.copy()
    for i in range(1,4):
        blur = cv2.GaussianBlur(costmap,(10*i-1,10*i-1),20)
        _, th = cv2.threshold(blur, 2, 255, cv2.THRESH_BINARY)        
        costmap = cv2.addWeighted(th, alpha, costmap, 1-alpha, 0.0)
    
    alpha = 0.6
    _, th = cv2.threshold(rotated, 245, 255, cv2.THRESH_BINARY)
    costmap = cv2.addWeighted(th, 1-alpha, costmap, alpha, 0.0)
    
    x_0, y_0 = [500,508]
    
    x_f = np.random.randint(487) + 270
    y_f = np.random.randint(448) + 363
    
    points = []
    path = cv2.cvtColor(skel,cv2.COLOR_GRAY2RGB)
    # path[x_0,y_0,:] = [0,0,255]
    # path[x_f,y_f,:] = [0,255,0]
    
    # dst = cv2.cornerHarris(skel,2,3,0.04)
    # dst = cv2.dilate(dst,None)
    # ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    # dst = np.uint8(dst)
    # # find centroids
    # ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    
    dst = cv2.cornerHarris(skel,2,5,0.04)
    # dst = np.uint8(dst)
    # ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # print centroids
    
    # kernel = cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3))
    # #dst = cv2.erode(dst, kernel)
    
    path[dst>0.0*dst.max()]=[0,0,255]

    # cv2.imshow("Original", image)
    # cv2.imshow("Rotada -7.66 grados", rotated)
    # cv2.imshow("Esqueleto", skel)
    # cv2.imshow("costmap", costmap)
    cv2.imshow("path", path)
    
    # cv2.imwrite('map/rotated.pgm', rotated)
    cv2.waitKey(0) 
    