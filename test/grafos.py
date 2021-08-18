#!/usr/bin/python
import numpy as np
import cv2
from matplotlib import pyplot as plt
import networkx as nx

G = nx.Graph()

points =[[1.0, 1.0],
         [3.0, 1.0],
         [-1.0, -0.5],
         [2.0, 3.0],
         [5.0, -1.0],
         [0.0, 0.5]]

for i, p in enumerate(points):
    G.add_node(i, pos=p)
    
    
print G.nodes.data('pos')

for i, p in G.nodes.data('pos'):
    print i, p

plt.figure()
nx.draw(G)

plt.show()