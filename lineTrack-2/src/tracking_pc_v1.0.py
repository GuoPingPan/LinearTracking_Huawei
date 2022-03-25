from signal import ITIMER_PROF
from xmlrpc.client import Binary
from numpy.core.numeric import binary_repr

from numpy.testing._private.utils import print_assert_equal
import rospy
from rospy.client import init_node
from rospy.core import logdebug_throttle_identical
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import cv2
import numpy as np
import time

'''
    这个文件是相机追踪的combine(只需要一个函数处理双边和单边的情况) python版本
    
    author: panguoping
    version:v1.0
    
'''


fx_ = 760.4862674784594
fy_ = 761.4971958529285
cx_ = 631.6715834996345
cy_ = 329.3054436037627
offset_ = 0.45

y_ = 129.5585
scale_ = 1000.0

path_pub = rospy.Publisher("cameraPath",PointCloud,queue_size=1)

'''
@brief 将图像两边线转化到三维坐标系下
@input [left_line_list(ndarray),left_line_list(ndarray)]
@output [left_world(ndarray),right_world(ndarray)]
'''
def img2world(left_line_2d,right_line_2d):

    def getXYZ(point):
        tmp_x = (fy_ * y_/(point[1]-cy_))/scale_
        tmp_y = (-(point[0]-cx_)*tmp_x/fx_)
        return [tmp_x ,tmp_y, 0.0 ]

    left_world = [ getXYZ(left_point) for left_point in left_line_2d ]
    right_world = [ getXYZ(right_point) for right_point in right_line_2d ]

    return left_world,right_world


'''
@brief 根据三维空间的两条车道线获得中间车道线轨迹
@input [left_world(ndarray),right_world(ndarray)]
@output path
'''
def getMiddleLine(left_world,right_world):

    r = 0
    l = 0
    pc = PointCloud()
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = 'camera'
    lastparallel = 0
    init = True
    # 左右匹配
    print(len(left_world))
    print(len(right_world))

    while(l<len(left_world) and r<len(right_world)):
        # 初始化   
        if(init):
            lastparallel = ((left_world[l][1]+right_world[r][1])/2)
            init = False
            continue
        if(left_world[l][0] == right_world[r][0]):  # 纵向距离相等
            # 前后点横坐标突变阈值 0.1m
            tmp =(left_world[l][1]+right_world[r][1])/2
            if(abs(tmp-lastparallel)>0.1): 
                l+=1
                r+=1
                # print("before continue")
                continue
            
            # print("after continue")    
            pc.points.append(Point32(left_world[l][0],
                                    tmp,
                                    0.0))
            lastparallel = tmp
            r+=1
            l+=1
        
        elif(left_world[l][0] > right_world[r][0] ):    # 左纵坐标距离远
            tmp = left_world[l][1]+offset_
            if(abs(tmp-lastparallel)>0.1):
                l+=1
                continue
            pc.points.append(Point32(left_world[l][0],
                                    tmp,
                                    0.0))
            lastparallel = tmp
            l+=1
        
        else:                                           # 右纵坐标距离远
            tmp = right_world[r][1]-offset_
            if(abs(tmp-lastparallel)>0.1):
                r+=1
                continue
            pc.points.append(Point32(right_world[r][0],
                        tmp,
                        0.0))
            lastparallel = tmp
            r+=1
    
    # 将尚未填入的点填入path
    while(l<len(left_world)):
        pc.points.append(Point32(left_world[l][0],
                                left_world[l][1]+offset_,
                                0.0))
        l+=1
    while(r<len(right_world)):
        pc.points.append(Point32(right_world[r][0],
                                right_world[r][1]-offset_,
                                0.0))
        r+=1

    path_pub.publish(pc)    


def showImg(img):
    cv2.imshow('img',img)
    cv2.waitKey(33)



def main():
    rospy.init_node("cameraPath",anonymous=True)    

    cap = cv2.VideoCapture('../dataset/01.mp4')
    
    while(not rospy.is_shutdown() and cap.isOpened()):
        t1 = time.time()
        ret , frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
        ret , binary = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)

        uv_left = []
        uv_right = []

        lup = int(binary.shape[0] * 2/3 -10)
        

        for i in range(lup,int(binary.shape[0]-1)):
            for j in range(int(binary.shape[1]//2),int(binary.shape[1]-10)):
                if(binary[i][j]==0 and binary[i][j+1]==255):
                    binary[i][j]=150
                    binary[i][j+1]=150
                    binary[i][j+2]=150
                    binary[i][j+3]=150
                    binary[i][j+4]=150
                    binary[i][j+5]=150
                    binary[i][j+6]=150
                    binary[i][j+7]=150
                    binary[i][j+8]=150
                    binary[i][j+9]=150
                    
                    uv_right.append([j,i])
                    break

        for i in range(lup,int(binary.shape[0]-1)):
            for j in range(int(binary.shape[1]//2),10,-1):
                if(binary[i][j]==0 and binary[i][j-1]==255):
                    binary[i][j]=150
                    binary[i][j-1]=150
                    binary[i][j-2]=150
                    binary[i][j-3]=150
                    binary[i][j-4]=150
                    binary[i][j-5]=150
                    binary[i][j-6]=150
                    binary[i][j-7]=150
                    binary[i][j-8]=150
                    binary[i][j-9]=150
                    uv_left.append([j,i])
                    break
        showImg(binary)
        
        print(len(uv_left))
        print(len(uv_right))

        getMiddleLine(img2world(uv_left,uv_right)[0],img2world(uv_left,uv_right)[1])

        print(f"使用时间(s):{time.time()-t1}")

if __name__=='__main__':

    main()





