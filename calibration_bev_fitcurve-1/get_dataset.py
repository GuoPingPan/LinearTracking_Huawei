#!-*-coding:utf-8-*-
import numpy as np
import cv2

#相机的行列数
cols = 640
rows = 480

# 获得相应的帧图像
def getDataset():
    cap = cv2.VideoCapture('./test.mp4')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cols)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rows)
    i = 0
    while(cap.isOpened()):
        ret,frame = cap.read()
        cv2.imshow('data',frame)
        key = cv2.waitKey(0)
        
        if key&0xff == ord('\n'):
        	continue
        
        if key&0xff == ord('s'):
            cv2.imwrite('./dataset/'+str(i)+'.png',frame)
            i+=1
            
        if key&0xff == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()


# 检查相机情况，并保存图像
def checkTheCamera():
    cap = cv2.VideoCapture('/dev/video10')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    save_path = ("please input the save_path: ")
    
    writer = cv2.VideoWriter(save_path, fourcc, 30, (cols, rows))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cols)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rows)
    while (cap.isOpened()):
        ret, frame = cap.read()
        cv2.imshow('cameraDown', frame)
        writer.write(frame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    writer.release()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    getDataset()
