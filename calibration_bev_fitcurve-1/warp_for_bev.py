import numpy as np
import cv2

'''
	该文件使用仿射变换来恢复鸟瞰图
'''


def check_camera_normal():
	cap = cv2.VideoCapture('/dev/video10')
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	writer = cv2.VideoWriter('./test.mp4',fourcc,30,(1280,720))
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	while(cap.isOpened()):	
		ret,frame = cap.read()
		cv2.imshow('cameraDown',frame)
		writer.write(frame)
		
		if cv2.waitKey(1)&0xff == ord('q'):
			break	
	cap.release()
	cv2.destroyAllWindows()


def warp_img(image,src,dst,size):
	image_size = (size[0],size[1])
	M = cv2.getPerspectiveTransform(src,dst)
	output = cv2.warpPerspective(image,M,image_size)
	return output



def rgb_warp():
	src = np.float32([[500,454],[754,454],[1004,715],[210,715]])
	dst = np.float32([[500,254],[754,251],[754,715],[500,715]])
	cap = cv2.VideoCapture('/dev/video10')
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	while(cap.isOpened()):	
		ret,frame = cap.read()
		converted = warp_img(frame,src,dst)
		cv2.imshow('cameraDown',converted)
		
		if cv2.waitKey(1)&0xff == ord('q'):
			break	
	cap.release()
	cv2.destroyAllWindows()


def gray_warp():
	src = np.float32([[500,454],[754,454],[1004,715],[210,715]])
	dst = np.float32([[500,254],[754,251],[754,715],[500,715]])
	cap = cv2.VideoCapture('/dev/video10')
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	while(cap.isOpened()):	
		ret,frame = cap.read()
		gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		ret,binary = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
		converted = warp_img(gray,src,dst,(frame.shape[1],frame.shape[0]))
		cv2.imshow('cameraDown',converted)
		
		if cv2.waitKey(1)&0xff == ord('q'):
			break	
	cap.release()
	cv2.destroyAllWindows()

def gray_warp_resize():
	# 改变仿射变换窗口大小
	src = np.float32([[500,454],[754,454],[1004,715],[210,715]])
	dst = np.float32([[210,1600],[1004,1600],[1004,3150],[210,3150]])
	cap = cv2.VideoCapture('/dev/video10')
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	while(cap.isOpened()):	
		ret,frame = cap.read()
		gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		ret,binary = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)

		# 调整输出的图片大小使得黑色填充部分减小
		converted = warp_img(gray,src,dst,(1280,3200))
		cv2.imshow('cameraDown',converted)
		
		if cv2.waitKey(1)&0xff == ord('q'):
			break	
	cap.release()
	cv2.destroyAllWindows()


if __name__=='__main__':
	gray_warp()
	
