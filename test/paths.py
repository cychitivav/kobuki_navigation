import numpy as np
import cv2

image = cv2.imread('map/map.pgm', 0)
_, umbral = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)        

kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (3,3)) 
apertura = cv2.morphologyEx(umbral, cv2.MORPH_OPEN, kernel)

skel = cv2.ximgproc.thinning(apertura)

cv2.imshow("Original", image)
cv2.imshow("Umbralizada", umbral)
cv2.imshow("Umbralizada filtrada", apertura)
cv2.imshow("Esqueleto", skel)
cv2.waitKey(0) 