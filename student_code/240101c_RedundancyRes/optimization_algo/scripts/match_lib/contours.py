#!/usr/bin/env python
# coding=latin-1
import os
import copy
import numpy as np
import math
import cv2 as cv
from matplotlib import path


def canny(depth_img, sureNoEdge, sureEdge):
    """Returns canny img as np.uint8

    Args:
        depth_img ([type]): [description]
        sureNoEdge ([type]): [description]
        sureEdge ([type]): [description]

    Returns:
        np.uint8: img with canny edges
    """
    edge_canny = cv.Canny(depth_img, sureNoEdge, sureEdge)  # Canny recommended a upper:lower ratio between 2:1 and 3:1
    edge_canny_abs = np.absolute(edge_canny)
    edge_canny_8 = np.uint8(edge_canny_abs)
    return edge_canny_8


def get_drawContours(img, drawtoimg=None, canny=False, thickness=1, color_in=None):
    if drawtoimg is None:
        drawtoimg = copy.copy(img)
    mask = cv.inRange(img, 5, 255)
    _, contours, hierachy = cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    depthtorgb = cv.cvtColor(drawtoimg, cv.COLOR_GRAY2RGB)

    for i in range(len(contours)):
        # if edge detection and there is a parent: next contur
        if canny and hierachy[0][i][3] != -1:
            continue
        
        if color_in is None:
            color = np.random.choice(range(256), size=3)
        else:
            color=color_in
        cv.drawContours(depthtorgb, contours, i, color, thickness)
        # cv.imshow("Contours", depthtorgb)
        # cv.waitKey(0)
    return depthtorgb


def contoursFromCannyEdge(img):
    mask = cv.inRange(img, 5, 255)
    # _ , contours, hierachy = cv.findContours(mask, cv.RETR_CCOMP,  cv.CHAIN_APPROX_SIMPLE)
    # _ , contours, hierachy = cv.findContours(mask, cv.RETR_CCOMP,  cv.CHAIN_APPROX_NONE)
    out = cv.findContours(mask, cv.RETR_CCOMP,
                          cv.CHAIN_APPROX_NONE)  # eig. hier 3 outputs? aktuelle version opencv?
    if len(out) == 2:
        contours, hierachy = out
    else:
        _, contours, hierachy = out

    contour_out = []
    closed = []
    for i in range(len(contours)):
        # if edge detection and there is a parent: next contur
        if hierachy[0][i][3] == -1:
            contour_out.append(contours[i])
            # for closed contours there is always a child when using edge detect
            if hierachy[0][i][2] == -1:
                closed.append(False)
            else:
                closed.append(True)

    return contour_out, closed


def drawContours(contours, drawtoimg, thickness=1):
    depthtorgb = cv.cvtColor(drawtoimg, cv.COLOR_GRAY2RGB)

    for i in range(len(contours)):
        color = np.random.choice(range(256), size=3)
        cv.drawContours(depthtorgb, contours, i, color, thickness)
        # cv.imshow("Contours", depthtorgb)
        # cv.waitKey(0)
    return depthtorgb


def calcAreaContour(contour):
    y_vals = contour[:, 0, 1]
    x_vals = contour[:, 0, 0]
    x_min = np.min(x_vals)
    x_max = np.max(x_vals)
    y_min = np.min(y_vals)
    y_max = np.max(y_vals)

    return (x_max - x_min) * (y_max - y_min)


def getBiggestContour(contours):
    return np.argmax([calcAreaContour(contour) for contour in contours])


def contourChildrenIdxAll(contours):
    """Get inner contours for all contours

    Args:
        contour 

    Returns:
        list[[idx_ChildsContour0], [idx_ChildsContour1],...]: returns list with idx of inner contours for each contour idx
    """
    childs = [[] for c in contours]
    for i in range(len(contours)):
        childs[i] = contourChildrenIdx(contours, i)
    return childs


def contourChildrenIdx(contours, idx):
    """Get inner Contours for contour at index 

    Args:
        contours ([type]): list of contours
        idx (int): parent contour

    Returns:
        list[idxs_ChildsContourIdx]: list with indexes of inner contours
    """
    childs = []
    outer_path = path.Path(contours[idx][:, 0])
    for i in range(len(contours)):
        inner_path = path.Path(contours[i][:, 0])
        if outer_path.contains_path(inner_path):
            childs.append(i)

    return childs


def getValsInContour(img, contour, child_contours=[]):
    """Returns Values inside contour minus child contours

    Args:
        img: img where to get values from
        contour (list): list[points]
        child_contours (list, optional): list[contours]. Defaults to [].

    Returns:
        np.array: values inside contour
    """
    # create mask inside contour
    cimg = np.zeros_like(img)
    cimg = cv.drawContours(cimg, [contour], 0, color=255, thickness=-1)
    
    # mask without child-contours
    for i in range(len(child_contours)):
        cimg = cv.drawContours(cimg, child_contours, i, color=0, thickness=-1)

    # get vals of img inside mask
    pts = np.where(cimg == 255)
    cont_vals = np.array(copy.copy(img[pts])).astype("float")
    cont_vals[cont_vals<=5] = np.NaN    # values never smaller 10
    
    # TODO: remove outsiders?
    
    return cont_vals

def getMinValInContour(img, contour, child_contours=[]):
    """Calc Minimum of img filled by Contour minus child contours.
        For grasp-dist: do not use child_contours to prevent collisions 

    Args:
        img: img where to get values from
        contour (list): list[points]
        child_contours (list, optional): list[contours]. Defaults to [].

    Returns:
        float: min value
    """
    # for grasp-dist: do not use child_contours to prevent collisions 
    cont_vals = getValsInContour(img, contour, child_contours)
    return np.nanmin(cont_vals)

def getAvgValueInContour(img, contour, child_contours=[]):
    """Calc Average filled by Contour minus child contours 

    Args:
        img: img where to get values from
        contour (list): list[points]
        child_contours (list, optional): list[contours]. Defaults to [].

    Returns:
        float: average value
    """
    cont_vals = getValsInContour(img, contour, child_contours)
    # avg = np.nansum(cont_vals)
    # avg /= len(cont_vals)
    avg = np.nanmedian(cont_vals)

    return avg


def getAvgValuesContours(contours_used, depth_img, minVal=3.0):
    """gets Average values for all contours (see getAvgValueInContour)

    Args:
        contours_used ([type]): [description]
        depth_img ([type]): image with depth values
        childsIdx
        minVal (float, optional): if smaller: returns Infinity as depth values of 0 are at inf. Defaults to 3.0.

    Returns:
        avg_vals_contours: Average Values of contours minus inside contours
        childsIdx (list[childIdxs]): output of contourChildrenIdxAll()
    """
    childsIdx = contourChildrenIdxAll(contours_used)

    avg_vals_contours = np.zeros(len(contours_used))
    for i, c in enumerate(contours_used):
        val = getAvgValueInContour(depth_img, c, contours_used[childsIdx[i]])
        if val <= minVal: val = np.Inf  # depth vals never smaller 10. so if lots of zero: depth at inf
        avg_vals_contours[i] = val

    return avg_vals_contours, childsIdx


def getContoursofBiggestObject(contours_closed, minArea=200.0):
    """Returns all contours inside biggest contour (boundary)

    Args:
        contours_closed ( list[contours]): contours in image
        minArea (float): smallest area to consider

    Returns:
        np.array[contours]: all contours inside boundary with the boundary at index 0
    """
    idx_biggest = getBiggestContour(contours_closed)  # boundary of objects

    # Get contours inside object:
    contours_used_idx = contourChildrenIdx(contours_closed, idx_biggest)
    contours_used_idx = np.insert(contours_used_idx, 0, idx_biggest).astype("int")
    # contours_used = contours_closed[contours_used_idx]

    contours_used = []
    for i in contours_used_idx:
        # Only if contour is big enough
        if calcAreaContour(contours_closed[i]) > minArea:
            contours_used.append(contours_closed[i])
    contours_used = np.array(contours_used)

    return contours_used

def getNearestContourWithHoles(img, sureNoEdge=10, sureEdge=50):
    """Gets contours by canny-edgeFilter,
    calculates nearestContour by average depth
    and returns children inside of nearestContour 

    Args:
        img ([type]): depth img
        sureNoEdge ([type]): for Canny
        sureEdge ([type]): for Canny

    Returns:
        np.array(contours): first idx is nearest contour, following are childContours
        np.array(): avg depth of contours
    """
    img_temp = copy.copy(img)
    maxX, maxY = img_temp.shape
    img_temp[0,:] = 0
    img_temp[maxY-1,:] = 0
    img_temp[:,maxX-1] = 0
    img_temp[:,0] = 0

    canny_img = canny(img_temp, sureNoEdge, sureEdge)

    contours_edgeDetect, _ = contoursFromCannyEdge(canny_img)
    if not len(contours_edgeDetect): return [], []
    contours_used = getContoursofBiggestObject(contours_edgeDetect)
    if not len(contours_used): return [], []
    # if not len(contours_used): continue
    avg_vals_contours, childsIdx = getAvgValuesContours(contours_used, img, 3.0)
    nearest_contourIdx = np.argmin(avg_vals_contours)
    contour_nearest = contours_used[nearest_contourIdx]
    # if object fills img:
    contour_nearest_real = [idxs for idxs in contour_nearest if idxs[0][0] != 0 and idxs[0][0] != maxY-2 and idxs[0][1]!=0 and idxs[0][1]!=maxX-2]
    contours_used[nearest_contourIdx] = np.array(contour_nearest_real)

    idx_contour_nearest_w_children = np.append(nearest_contourIdx, childsIdx[nearest_contourIdx])

    return contours_used[idx_contour_nearest_w_children], avg_vals_contours[idx_contour_nearest_w_children]

def verticalPartOfContour(contour, eps=0.1):
    """Returns only the vertical lines on contour, refined with intermediate points (max: 45deg)

    Args:
        contour (contours[i]): output of cv.findContours()[i]
        eps (float): for approximation of contour (epsilon=eps*arcLength)  

    Returns:
        list of contour parts dim=(n,m,1,2)
    """
    epsilon = eps*cv.arcLength(contour,True)
    approx = cv.approxPolyDP(contour,epsilon,True)

    # Delete horizontal segments um Kanten parallel zu Normalen der Greifflaechen zu ignorieren
    approx_out = []
    lastSplitIdx=0
    def isHorizontal(p1, p2):
        delta = (p2 - p1)[0]
        if abs(delta[0]) > abs(delta[1]):   # or atan
            return True
        else:
            return False
    
    def isSameContourPoint(p1,p2):
        return True if all((p1 == p2)[0]) else False

    for idx in range(len(approx)-1):        
        if isHorizontal(approx[idx], approx[idx+1]):
            # split
            splitIdx = idx+1
            approx_tmp = approx[lastSplitIdx:splitIdx]
            print("approx Temp: "+approx_tmp.__str__())
            lastSplitIdx = splitIdx
            if len(approx_tmp)>1:
                approx_out.append(approx_tmp)

    # append remaining points:
    approx_tmp = approx[lastSplitIdx:]
    if len(approx_tmp)>1:
        approx_out.append(approx_tmp)

    # connect last points to first points of list:
    p1,p2 = approx[idx+1], approx[0]
    if not isHorizontal(p1,p2):
        if isSameContourPoint(p1, approx_out[-1][-1]):
            if isSameContourPoint(approx_out[0][0], p2):
                # append last segments to first
                approx_out[0] = np.append(approx_out[-1], approx_out[0],0)
                approx_out = approx_out[:-1]
            else:
                # append first point to last segment
                approx_out[-1] = np.append(approx_out[-1], p2,0)
        else:
            if isSameContourPoint(approx_out[0][0], p2):
                # append last element to first segment
                approx_out[0] = np.append(p1, approx_out[0],0)
            else:
                # add segment with the two points
                approx_out.append(np.array([p1,p2]))
    
    return approx_out



        # refine
def addIntermediatePoints(p1,p2,maxD):
    line_out=[p1]
    delta_total = p2-p1
    numPoints = np.linalg.norm(delta_total)/maxD
    delta = delta_total/numPoints
    numPoints = int(numPoints)
    for i in range(numPoints):
        nextPoint = line_out[-1]+delta
        line_out.append(nextPoint)
    if not all((line_out[-1] == p2)[0]):
        line_out.append(p2)
    return line_out

def refineContour(contour, maxD):
    contour_out = np.zeros((0,1,2))
    for idx in range(len(contour)-1):
        contour_out = np.append(contour_out, addIntermediatePoints(contour[idx],contour[idx+1],maxD), 0).astype('int')
    return contour_out


if __name__ == "__main__":
    #img = cv.imread(os.path.dirname(__file__) + "/img/Depth.png", cv.IMREAD_GRAYSCALE)
    max_size = 400
    img = np.zeros((max_size,max_size)).astype('uint8')
    img[0:max_size, int(max_size/2):max_size] = 255
    img[int(max_size/4):int(max_size/4*3), int(max_size/4*3):int(max_size/8*7)] = 0

    contours_used, _ = getNearestContourWithHoles(img)
    contour_img = drawContours(contours_used, img)
    cv.imshow("Contours ohne Rand", contour_img)
    cv.waitKey(0)
