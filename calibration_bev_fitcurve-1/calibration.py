'''
    该文件使用opencv棋盘法标定相机
'''
import cv2
import numpy as np

def calibarte():

    # 首先读取图像并转为灰度图
    img = cv2.imread('./calib/1.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 5),None)
    # print(ret)
    # print(corners)  # 交点坐标

    if ret == True:
        img = cv2.drawChessboardCorners(img, (9, 5), corners, ret)


    cv2.imshow("result",img)

    # 构造这些对角点在在现实世界中的相对位置，我们将这些位置简化成整数值
    objp = np.zeros((5*9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:5].T.reshape(-1, 2)

    # 这里之所以还要再搞一个list去装是因为他可以装多张图片
    img_points = []
    obj_points = []
    img_points.append(corners)
    obj_points.append(objp)

    # 最后我们使用OpenCV中的 cv2.calibrateCamera() 即可求得这个相机的畸变系数，在后面的所有图像的矫正都可以使用这一组系数来完成
    image_size = (img.shape[1], img.shape[0])
    #  It returns the camera matrix, distortion coefficients, rotation and translation vectors etc.
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points,image_size, None, None)

    test_img = cv2.imread("./calib/1.png")
    # 这里cv2.undistort的最后一个参数依旧选用原来的相机矩阵mtx
    # 也可以尝试做opencv官网举例的使用cv2.getOptimalNewCameraMatrix（）基于自由缩放参数来优化相机矩阵，之后再带入
    undist = cv2.undistort(test_img, mtx, dist, None, mtx)

    cv2.imshow("test_img",test_img)
    cv2.imshow("undist",undist)

    cv2.waitKey()
    cv2.destroyAllWindows()

# def calib2():
#     patternSize = np.array([11,4])
#     cap = cv2.VideoCapture()
#     retval,frame = cap.read()
#     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     if (cap.isOpened()):
#         ret,coners = cv2.findCirclesGrid(frame,patternSize,flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
#         if ret:
#             cv2.cornerSubPix(gray,patternSize)

if __name__ == '__main__':
    calib1()