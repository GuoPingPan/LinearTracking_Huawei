'''
	该文件用于检测摄像头是否正常

	Tip:
		注意修改VideoCapture的端口
'''

import numpy as np
import cv2

def check_camera_normal():
	cap = cv2.VideoCapture('/dev/video10')
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	writer = cv2.VideoWriter('./test.mp4',fourcc,30,(640,480))
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
	while(cap.isOpened()):	
		ret,frame = cap.read()
		cv2.imshow('cameraDown',frame)
		writer.write(frame)
		
		if cv2.waitKey(1)&0xff == ord('q'):
			break	
	cap.release()
	cv2.destroyAllWindows()

if __name__=='__main__':
	check_camera_normal()
	
