#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import cv2
from skimage.io import imread
from skimage.color import rgb2gray
from scipy import ndimage

def voronoi(points,shape=(500,500)):
    depthmap = np.ones(shape,np.float)*1e308
    colormap = np.zeros(shape,np.int)

    for i,(x,y) in enumerate(points):
        hypot = lambda (X,Y): (X-x)**2 + (Y-y)**2

        paraboloid = np.fromfunction(hypot,shape)
        colormap = np.where(paraboloid < depthmap,i+1,colormap)
        depthmap = np.where(paraboloid < depthmap,paraboloid,depthmap)

    for (x,y) in points:
        colormap[x-1:x+2,y-1:y+2] = 0.0

    return colormap

img = cv2.imread('maps/map.pgm')

rotated = ndimage.rotate(img, -8)
resized = cv2.resize(rotated, (1024,1024), interpolation = cv2.INTER_AREA)


nimg = cv2.cvtColor(rotated, cv2.COLOR_RGB2GRAY)

cv2.imwrite('maps/rotated.pgm',nimg)


plt.imshow(img)

plt.figure()
plt.imshow(rotated)

plt.show()