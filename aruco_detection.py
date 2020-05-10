'''
检测图像中的ArUco码并获取其顶点坐标和ID
被检测图像保存目录需与本文件相同
Detect the ArUco code in the image and obtain its vertex coordinates and ID
The saved directory of the detected image should be the same as this file
'''
import numpy as np  
import cv2  
import cv2.aruco as aruco

img = cv2.imread('aruco.png')
aruco_dict = cv2.aruco_Dictionary.get(aruco.DICT_6X6_250)
parameters = cv2.aruco_DetectorParameters.create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
print(corners)
print(ids)
result = cv2.aruco.drawDetectedMarkers(img, corners, ids)
cv2.imwrite('result.png',result)