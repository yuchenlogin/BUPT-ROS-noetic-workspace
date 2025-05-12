#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import sys

RED_RANGE_UPPER = [16, 255, 240]
RED_RANGE_LOWER = [0, 224, 96]
RED_RANGE = (RED_RANGE_LOWER, RED_RANGE_UPPER)
SINGLE_MODEL_WIDTH = 250
POINT_X_INDEX = 0
DIGIT_1 = 1
DIGIT_2 = 2
FIFTY_PRECENT = 0.5

class digits_recognition():
    def __init__(self, origin_image, model_image, debug=False):
        self.__origin_image = origin_image
        self.__model_image = model_image
        self.__debug = debug

    def image_cropping(self, origin_image):
        blurred = cv.GaussianBlur(origin_image, (5, 5), 0)
        hsvImg = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsvImg, np.array(RED_RANGE[0]), np.array(RED_RANGE[1]))
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.erode(mask, None, iterations=2)
        contours = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        if len(contours) > 0:
            c = max(contours, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(c)
        roi_image = origin_image[y : y + h, x : x + w]
        return roi_image

    def feature_extraction(self, image, model_image):
        gray1 = cv.cvtColor(model_image, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        sift = cv.xfeatures2d.SIFT_create()
        kp1, psd_des1 = sift.detectAndCompute(gray1, None)
        kp2, psd_des2 = sift.detectAndCompute(gray2, None)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(check=50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(psd_des1, psd_des2, k=2)
        good_matches = []
        matche_points = []
        for m, n in matches:
            if m.distance < 0.5*n.distance:
                good_matches.append(m)
                matche_points.append(kp1[m.queryIdx].pt)
        good_matches = np.expand_dims(good_matches, 1)
        img_out = cv.drawMatchesKnn(model_image, kp1, image, kp2, good_matches[:20], None, flags=2)
        if self.__debug == True:
            self.show_image(img_out)
        leftpoints_Cnt = 0
        for ptx in matche_points:
            if ptx[POINT_X_INDEX] < SINGLE_MODEL_WIDTH:
                leftpoints_Cnt += 1
        return DIGIT_1 if float(leftpoints_Cnt)/float(len(matche_points)) > FIFTY_PRECENT else DIGIT_2
            
    def drawmatches(self, img1, img2, kp1, kp2, matches):
        outimg = cv.drawMatches(img1, kp1, img2, kp2, matches, outImg=None)
        return outimg
    
    def show_image(self, image):
        cv.imshow('debug image', image)
        cv.waitKey(0)

    def main(self):
        roi_image = self.image_cropping(self.__origin_image)
        feature_extraction_result = self.feature_extraction(roi_image, self.__model_image)
        return feature_extraction_result

if __name__ == "__main__":
    ori_image = cv.imread('digits_model/test8.jpg')
    model_image = cv.imread('digits_model/right_model.png')
    
    if len(sys.argv) > 1 and sys.argv[1] == 'debug':
        d = digits_recognition(ori_image, model_image, debug=True)
    else:
        d = digits_recognition(ori_image, model_image)
        
    process_result = d.main()
    print(process_result)
