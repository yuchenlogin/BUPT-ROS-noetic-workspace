#!/usr/bin//python

import cv2 as cv
import sys
import numpy as np
import os

current_working_directory = os.getcwd()
execution_file_path = sys.argv[0]
execution_file_dir_path = os.path.dirname(os.path.join(current_working_directory, execution_file_path))

def image_read(args):
    return [cv.imread(arg) for arg in args]

def synthetic_picture(image_list):
    return np.concatenate(image_list, axis=1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        image_list = image_read(sys.argv[2:])
        picture_syntheticed = synthetic_picture(image_list)
        if sys.argv[1] == 'save':
            cv.imwrite('digits_model/new.png', picture_syntheticed)
        elif sys.argv[1] == 'show':
            cv.imshow('synthetic result', picture_syntheticed)
            cv.waitKey(0)
    else:
        print('Please enter the path of at least one picture!')
