import numpy as np
import cv2
import time

'''
    该文件用于在Windows测试使用Python完成到路线检测任务
    使用飞思卡尔曲线拟合的角度去计算车道线曲线
'''

# 图像尺寸
imWidth = 640
imHeigt = 480


def capture_video_cv2():
    '''
        使用cv2来捕获相机帧并显示
    '''
    cap = cv2.VideoCapture()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, imWidth)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, imHeigt)
    while (cap.isOpened()):
        ret, frame = cap.read()

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    cap.release()
    cv2.destroyWindow()


def get_boundary():
    '''
        使用图片测试提取左右边界效果
    '''
    frame = cv2.imread('./calib/4.jpg')
    imHeigt = frame.shape[0]
    imWidth = frame.shape[1]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    cv2.imshow('4', binary)
    cv2.waitKey()

    pmiddle = int(imHeigt / 2)
    vmiddle = int(imWidth / 2)
    print(pmiddle, vmiddle)

    # 提取行数选择
    parallelLines = []
    for i in range(pmiddle + 100, imHeigt, 100):
        parallelLines.append(i)
    print(parallelLines)

    leftFlag = []
    rightFlag = []

    # 提取左边界
    for i, line in enumerate(parallelLines):
        for j in range(vmiddle, 0, -1):
            print(i, j)
            if binary[line][j] == 0 and binary[line][j - 1] == 255:
                leftFlag.append(j)
        if len(leftFlag) == i:
            leftFlag.append(-1)

    # 提取右边界
    for i, line in enumerate(parallelLines):
        for j in range(vmiddle, imWidth - 1, 1):
            print(i, j)
            if binary[line][j] == 0 and binary[line][j + 1] == 255:
                rightFlag.append(j)
        if len(rightFlag) == i:
            rightFlag.append(-1)

    print(leftFlag)
    print(rightFlag)

def calculateR(y,w):
    return pow(1+pow(2*w[0]*y+w[1],2),1.5)/(2*w[0])

def fit_curve():
    t1 = time.time()
    frame = cv2.imread('./calib/4.jpg')
    imHeigt = frame.shape[0]
    imWidth = frame.shape[1]
    # 二值化转曲率
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    binary = cv2.erode(binary,None,iterations=2)
    # binary = cv2.dilate(binary,None)

    # 图的中点
    pmiddle = int(imHeigt / 2)
    vmiddle = int(imWidth / 2)

    # 提取右边界
    rightSize = []
    for i in range(pmiddle,imHeigt-1):
        binary[i][vmiddle] = 150
    for i in range(pmiddle,imHeigt-1):
        for j in range(vmiddle,imWidth-1):
            if binary[i][j] == 0 and binary[i][j+1] == 255:
                rightSize.append([i,j])
                binary[i][j] = 150
                binary[i][j-1] = 150
                binary[i][j+1] = 150
                binary[i-1][j-1] = 150
                binary[i-1][j+1] = 150
                binary[i+1][j-1] = 150
                binary[i+1][j+1] = 150
                binary[i+1][j] = 150
                binary[i-1][j] = 150

    # 中点往下的每条线的横坐标偏差
    print(rightSize)
    tmp = np.array(rightSize)
    rightDelta = tmp.copy()
    rightDelta[:,1] -= vmiddle


    '''         尝试拟合曲率          '''
    # tmp = tmp[np.where(rightDelta[:,1]>200 )]
    # print(X)
    # print(np.square(tmp[:,0]))
    # X = tmp[:,1]
    # Y = np.c_[np.square(tmp[:,0]),tmp[:,0]]
    # Y = np.insert(Y,2,values=1,axis=1)
    # print(Y)
    #
    # # print(rightDelta)
    # # print(Y.T.shape)
    # # print(X.shape)
    # # print(Y.T.dot(Y))
    # w = np.linalg.inv(Y.T.dot(Y)).dot(Y.T.dot(X))
    # print(w)
    # print(calculateR(600,w))

    ''' python 计算方程 '''
    # z = np.polyfit(tmp[:,0],tmp[:,1],2)
    # p = np.poly1d(z)
    # print(z)
    # print(p)
    # print(calculateR(600,z))
    print(time.time()-t1)
    cv2.imshow('binary',binary)
    cv2.waitKey(0)
    '''-----------------------------------'''

if __name__ == '__main__':
    fit_curve()
