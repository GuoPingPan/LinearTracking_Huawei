import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import cv2
import numpy as np

'''
    这个文件是相机追踪的combine(只需要一个函数处理双边和单边的情况) python版本
    
    author: panguoping
    version:v2.0
    
'''

fx_ = 760.4862674784594
fy_ = 761.4971958529285
cx_ = 631.6715834996345
cy_ = 329.3054436037627
offset_ = 0.45

y_ = 129.5585
scale_ = 1000.0

path_pub = rospy.Publisher("cameraPath",PointCloud,queue_size=1)


def img2world(left_line_2d,right_line_2d):
    '''将图像两边线转化到三维坐标系下

    通过利用相机模型公式进行变换,详情见getXYZ
    
    Args:
        left_line_2d(list): the 2d line in the left of the image
        right_line_2d(list): the 2d line in the right of the image
   
    Returns:
        left_world(list): the 3d line in the left of the world
        right_world(list): the 3d line in the right of the world
    '''
    def getXYZ(point):
        '''将二维点映射到三维空间
        
        Args:
            point(u,v): the point in image
        
        Theory:
            u = fx/z * x + cx
            v = fx/z * y + cy
        Returns:
            point3d(x,y,0)
        '''
        tmp_x = (fy_ * y_/(point[1]-cy_))/scale_
        tmp_y = (-(point[0]-cx_)*tmp_x/fx_)
        return [tmp_x ,tmp_y, 0.0 ]

    left_world = [ getXYZ(left_point) for left_point in left_line_2d ]
    right_world = [ getXYZ(right_point) for right_point in right_line_2d ]

    return left_world,right_world



def getMiddleLine(left_world,right_world):
    '''根据三维空间的两条车道线获得中间车道线轨迹
    
    Args:
        left_world(list): the 3d line in the left of the world
        right_world(list): the 3d line in the right of the world

    Outputs:
        path(PointCloud): the middle line for the car to drive
    '''

    r = 0
    l = 0
    pc = PointCloud()
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = 'camera'
    while(l<len(left_world)-1 and r<len(right_world)-1):
        if(left_world[l][0] == right_world[r][0]):
            pc.points.append(Point32(left_world[l][0],
                                    ((left_world[l][0]+right_world[r][0])/2),
                                    0.0))
            r+=1
            l+=1
        elif(left_world[l][0] > right_world[r][0] ):
            pc.points.append(Point32(left_world[l][0],
                                    left_world[l][0]+offset_,
                                    0.0))
            l+=1
        else:
            pc.points.append(Point32(right_world[r][0],
                        right_world[r][0]-offset_,
                        0.0))
            r+=1
    if(l<len(left_world)-1):
        while(l<len(left_world)-1):
            pc.points.append(Point32(left_world[l][0],
                                    left_world[l][0]+offset_,
                                    0.0))
            l+=1
    if(r<len(right_world)-1):
            pc.points.append(Point32(right_world[r][0],
                right_world[r][0]-offset_,
                0.0))
            r+=1

    path_pub.publish(pc)    


def showImg(img):
    print(type(img))
    cv2.imshow('img',img)
    cv2.waitKey(33)

def main():
    rospy.init_node("cameraPath",anonymous=True)    

    cap = cv2.VideoCapture('../dataset/output.avi')
    
    while(not rospy.is_shutdown() and cap.isOpened()):
        ret , frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
        ret , binary = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
        print(frame.shape)
        showImg(binary)

        uv_left = []
        uv_right = []

        for i in range(binary.shape[0]//2,int(binary.shape[0]*2/3)):
            for j in range(binary.shape[1]//2,binary.shape[1]-1):
                if(binary[i][j]==0 and binary[i][j+1]==255):
                    uv_right.append([j,i])
            

        for i in range(binary.shape[0]//2,binary.shape[0]*2/3):
            for j in range(binary.shape[1]//2,0,-1):
                if(binary[i][j]==0 and binary[i][j-1]==255):
                    uv_left.append([j,i])

        getMiddleLine(img2world(uv_left,uv_right))


if __name__=='__main__':

    main()





