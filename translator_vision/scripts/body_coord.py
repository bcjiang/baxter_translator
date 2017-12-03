#!/usr/bin/env python
import cv2
import numpy as np

def getCentroid(img):
    (col_list, row_list) = np.where(img == 0)
    centroid_row = np.sum(row_list)/(row_list.size)
    centroid_col = np.sum(col_list)/(col_list.size)
    return centroid_row, centroid_col

def body_coord(depth_image):
    # threshold the depth image to get human shape
    retval2, threshold = cv2.threshold(depth_image, 125, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # find centroid of detected human
    threshold = np.array(threshold)
    body_coord_row, body_coord_col = getCentroid(threshold)
    return (body_coord_row, body_coord_col)

def position_3D(point, depth, frame):
    (row, col) = np.subtract(point, frame)
    return (row, col, depth)

def plot_vector(img, point1, point2):
    # plot a vector from point2 to point1
    cv2.circle(img, point1, 5, (0,255,0), 4)
    cv2.line(img, point1, point2, (0,255,0),2)
    return img