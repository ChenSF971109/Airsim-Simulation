'''
相机：0
需与auto.py结合使用
顺序：(1)target_tracking.py(2)auto.py
Camera:0
Need to be used in combination with auto.py
Order:(1)target_tracking.py(2)auto.py
'''
import airsim
import time
import pprint
import os
import tempfile
import numpy as np
import cv2
import cv2.aruco as aruco
import socket
from functools import reduce


#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#相机图片大小
#camera picture size
image_width = 1080
image_height = 720

#暂停等待连接
#pause and waiting for connection
sk = socket.socket()
sk.bind(('127.0.0.1',8090))
sk.listen(5)
fd,addr = sk.accept()

file = open('camera_yaw.txt', 'r')
camera_yaw = float(file.read().strip())
file.close
camera_yaw_max = camera_yaw + 0.1
camera_yaw_min = camera_yaw - 0.1

while True:
    #读取当前角度
	#read current angles
    file = open('camera_yaw.txt', 'r')
    camera_yaw = float(file.read().strip())
    file.close
    file = open('camera_pitch.txt', 'r')
    camera_pitch = float(file.read().strip())
    file.close

    print(camera_yaw)
    print(camera_pitch)

    #保存相机图片
	#save the camera images
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene),
        airsim.ImageRequest("3", airsim.ImageType.Scene), 
        airsim.ImageRequest("4", airsim.ImageType.Scene)]) 
    print('Retrieved images: %d' % len(responses))
	
    tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
    print ("Saving images to %s" % tmp_dir)
    try:
        os.makedirs(tmp_dir)
    except OSError:
        if not os.path.isdir(tmp_dir):
            raise
	
    for idx, response in enumerate(responses):

        filename = os.path.join(tmp_dir, str(idx))
    
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
            img_rgb = img1d.reshape(response.height, response.width, 3)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)
    
	#识别ArUco
	#detect the ArUco code
    img = cv2.imread('../Appdata/Local/Temp/airsim_drone/0.png')
    aruco_dict = cv2.aruco_Dictionary.get(aruco.DICT_6X6_250)
    parameters = cv2.aruco_DetectorParameters.create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    
	#计算目标坐标偏移量
	#calculate the target coordinate offsets
    x1 = corners[0][0][0][0]
    y1 = corners[0][0][0][1]
    x2 = corners[0][0][1][0]
    y2 = corners[0][0][1][1]
    x3 = corners[0][0][2][0]
    y3 = corners[0][0][2][1]
    x4 = corners[0][0][3][0]
    y4 = corners[0][0][3][1]
    center_x = (x1 + x2 + x3 + x4)/4
    center_y = (y1 + y2 + y3 + y4)/4
    print(center_x,center_y)
    dx = center_x - image_width/2
    dy = center_y - image_height/2
    
	#根据偏移调整相机角度
	#adjust the camera angles according to the offsets
    if dx > 0:
        camera_yaw = camera_yaw + 0.001
    else:
        camera_yaw = camera_yaw - 0.001
    if dy > 0:
        camera_pitch = camera_pitch - 0.002
    else:
        camera_pitch = camera_pitch + 0.002	
	
    #yaw限幅
	#yaw limiting
    if camera_yaw > camera_yaw_max:
        camera_yaw = camera_yaw_max
    elif camera_yaw < camera_yaw_min:
        camera_yaw = camera_yaw_min
    
	#写入新的角度
	#write the new angles
    file = open('camera_yaw.txt', 'w')
    file.write(str(camera_yaw))
    file.close
    file = open('camera_pitch.txt', 'w')
    file.write(str(camera_pitch))
    file.close
	
	#重新调整相机
	#readjust the camera
    client.simSetCameraOrientation(0, airsim.to_quaternion(camera_pitch, 0, camera_yaw))


