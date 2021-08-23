#!/usr/bin/python
import numpy as np
import cv2
from matplotlib import pyplot as plt
import networkx as nx


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
    #cv2.imwrite('map/rotated.pgm', rotated)
    
    _, th = cv2.threshold(rotated, 245, 255, cv2.THRESH_BINARY)  
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)) 
    op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
    skel = cv2.ximgproc.thinning(op)  
    
    plt.figure()
    plt.subplot(1,3,1)
    plt.imshow(image, cmap='gray')
    plt.axis('off')
    plt.title('Original')

    plt.subplot(1,3,2)
    plt.imshow(rotated, cmap='gray')
    plt.axis('off')
    plt.title('Rotada')

    plt.subplot(1,3,3)
    plt.imshow(skel, cmap='gray')
    plt.axis('off')
    plt.title('Adelgazada')
    
    
    base = cv2.dilate(skel, None, iterations=12)
    path = cv2.cvtColor(base, cv2.COLOR_GRAY2RGB)     
    
    corners = cv2.cornerHarris(skel,7,7,0.04)   
    corners = cv2.dilate(corners, None)
    _, corners = cv2.threshold(corners,0.001,255,cv2.THRESH_BINARY)
    corners = np.uint8(corners)
    
    contours, _ = cv2.findContours(corners,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    path[corners>0.0]=[0,255,0]
    cv2.drawContours(path,contours,-1,(255,0,0),1)

    G = nx.Graph()     
    points = []
    for i, c in enumerate(contours):
        # calculate moments for each contour
        M = cv2.moments(c)
        
        # calculate x,y coordinate of center
        cX = int(round(M["m10"] / M["m00"]))
        cY = int(round(M["m01"] / M["m00"]))
        
        path[cY,cX]=[0,0,255]
        G.add_node(i, pos=(cX,cY))
        points.append((cX,cY))
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.4
        fontColor              = (0,0,255)
        thickness              = 1
        
        path = cv2.putText(path, str(i), (cX,cY), font, fontScale, fontColor, thickness)
        
    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(base,cmap='gray')
    plt.axis('off')
    plt.title('Imagen base')
    
    plt.subplot(1,2,2)
    plt.imshow(path)
    plt.axis('off')
    plt.title('Esquinas')
    
    
    
    noBlack = cv2.countNonZero(cv2.cvtColor(path,cv2.COLOR_BGR2GRAY))
    for i, p1 in enumerate(points):
        for j, p2 in enumerate(points):
            if p1 == p2: continue
            
            test_img = cv2.line(path.copy(), p1, p2, (234,0,234), 1)
            
            # Recount to see if the images are the same
            if cv2.countNonZero(cv2.cvtColor(test_img,cv2.COLOR_BGR2GRAY)) == noBlack: 
                # path = cv2.line(path, p1, p2, (234,0,234), 1)
                G.add_edge(i,j,weight=np.hypot(p1[0]-p2[0], p1[1]-p2[1]))
                
    plt.figure()
    nx.draw(G,with_labels=True)
    
    x_0, y_0 = [492,500]
    
    x_f = np.random.randint(487) + 277
    y_f = np.random.randint(448) + 368
    
    path[y_0+1,x_0+1] = (255,0,0)
    path[y_f+1,x_f+1] = (255,0,0)
    
    _, th = cv2.threshold(rotated, 245, 255, cv2.THRESH_BINARY)   
    
    ero = cv2.erode(th,None,iterations=10)
    
    th = ero.copy()
    noBlack = cv2.countNonZero(th)
    for i, p in enumerate(points):
        test_img = cv2.line(th.copy(), (x_0,y_0), p, 234, 1)
        
        # Recount to see if the images are the same
        if cv2.countNonZero(test_img) == noBlack: 
            # path = cv2.line(path, p1, p2, (234,0,234), 1)
            G.add_edge('p_0',i,weight=np.hypot(p[0]-x_0, y_0-p[1]))
            
    for i, p in enumerate(points):
        test_img = cv2.line(th.copy(), (x_f,y_f), p, 234, 1)
        
        # Recount to see if the images are the same
        if cv2.countNonZero(test_img) == noBlack: 
            # path = cv2.line(path, p1, p2, (234,0,234), 1)
            G.add_edge('p_f',i,weight=np.hypot(p[0]-x_f, y_f-p[1]))
    
    
    
    plan = nx.shortest_path(G,'p_0','p_f')
    print plan
    
    for i in range(len(plan)-1):
        if i == 0:
            path = cv2.line(path, (x_0,y_0), points[plan[i+1]], (251,229,78), 1)
        elif i == len(plan)-2:
            path = cv2.line(path, points[plan[i]], (x_f,y_f), (251,229,78), 1)
        else:
            path = cv2.line(path, points[plan[i]], points[plan[i+1]], (251,229,78), 1)

    plt.figure()
    plt.imshow(ero,cmap='gray')
    plt.axis('off')
    plt.title('Imagen erosionada')
    plt.show() 
    