#!/usr/bin/python
import numpy as np
import cv2
from matplotlib import pyplot as plt
import networkx as nx


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
                    im[i,j] = 0
    
    return im

    

if __name__ == "__main__":   
    image = cv2.imread('map/map.pgm', 0)
    rotated = rotate_image(image, -7.66)
    skel = get_skeleton(image)
    
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
    
    base = cv2.dilate(skel,None,iterations=4)
    path = cv2.cvtColor(base,cv2.COLOR_GRAY2RGB)
    # path[x_0,y_0,:] = [0,0,255]
    # path[x_f,y_f,:] = [0,255,0]
    
    
    corners = cv2.cornerHarris(skel,3,3,0.042)   
    corners = cv2.dilate(corners, None)
    _, corners = cv2.threshold(corners,0.01*corners.max(),255,cv2.THRESH_BINARY)
    corners = np.uint8(corners)
    
    contours, _ = cv2.findContours(corners,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    
    # path[corners>0.0]=[0,255,0]
    # cv2.drawContours(path,contours,-1,(255,0,0),1)

    points = []
    for i, c in enumerate(contours):
        # calculate moments for each contour
        M = cv2.moments(c)
        
        # calculate x,y coordinate of center
        cX = int(round(M["m10"] / M["m00"]))
        cY = int(round(M["m01"] / M["m00"]))
        
        path[cY,cX]=[0,0,255]
        points.append((cX,cY))
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.4
        fontColor              = (0,0,255)
        thickness              = 1
        
        path = cv2.putText(path, str(i), (cX,cY), font, fontScale, fontColor, thickness)
        
        
    
    G = nx.Graph()
        
    noBlack = cv2.countNonZero(cv2.cvtColor(path,cv2.COLOR_BGR2GRAY))
    for i, p1 in enumerate(points):
        for j, p2 in enumerate(points):
            if p1 == p2: continue
            
            test_img = cv2.line(path.copy(), p1, p2, (234,0,234), 1)
            
            # Recount to see if the images are the same
            if cv2.countNonZero(cv2.cvtColor(test_img,cv2.COLOR_BGR2GRAY)) == noBlack: 
                # path = cv2.line(path, p1, p2, (234,0,234), 1)
                G.add_edge(i,j,weight=np.hypot(p1[0]-p2[0], p1[1]-p2[1]))
                
    plan = nx.shortest_path(G,30,19)
    
    for i in range(len(plan)-1):
        path = cv2.line(path, points[plan[i]], points[plan[i+1]], (251,229,78), 3)

    # cv2.imshow("Original", image)
    # cv2.imshow("Rotada -7.66 grados", rotated)
    # cv2.imshow("Esqueleto", skel)
    # cv2.imshow("costmap", costmap)
    cv2.imshow('path',path)
    #cv2.imshow("d",base_img)
    
    # cv2.imwrite('map/rotated.pgm', rotated)
    cv2.waitKey() 
    