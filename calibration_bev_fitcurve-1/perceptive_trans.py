'''
    该文档记录的是opencv的透视变换用法
'''

import numpy as np
import cv2


def warp_img(image,src,dst):
    image_size = (image.shape[1],image.shape[0])
    print(image_size)
    M = cv2.getPerspectiveTransform(src,dst)
    invM = cv2.getPerspectiveTransform(dst,src)
    output = cv2.warpPerspective(image,M,image_size,flags=cv2.INTER_LINEAR)
    return output

src = np.float32([[267, 313],[393,127 ],[393, 596],[267, 406]])
dst = np.float32([[267, 100],  [393, 100], [393, 600],[267, 600]])

src = np.float32([[313,267],[406,267],[596,393],[127,393]])
dst = np.float32([[100,267],[600,267],[600,393],[100,393]])



if __name__ =='__main__':
    image = cv2.imread('./calib/9.png')
    #src = np.float32([[526,430],[734,430],[1004,715],[210,715]])
    src = np.float32([[500,454],[754,451],[1004,715],[210,715]])
    dst = np.float32([[500,254],[754,251],[754,715],[500,715]])
    cv2.imshow('9',image)
    cv2.waitKey()
    output = warp_img(image,src,dst)
    cv2.imshow('warp',output)
    cv2.imwrite('./calib/o2.png',output)
    cv2.waitKey()