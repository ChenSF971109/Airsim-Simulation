'''
相机：0
估计相对位置由无人机相机高度、云台角度和像素点坐标偏移量计算
*使用了Python多线程，包括三个线程：1、数据处理 2、云台 3、无人机飞行
*加入了GPS数据和IMU数据的卡尔曼滤波融合，GPS与PBVS融合采用根据传感器测量不确定度加权的方法
*云台控制方法使用PID控制，输入量为目标图像的偏移量
无人机基本到达降落平台正上方时只根据像素点坐标来调整
Camera:0
The estimated relative position is calculated by the height of the UAV camera, PTZ angles and the pixel coordinate offsets 
*Python multithreading is used, including three threads: 1.Data processing 2.PTZ 3.UAV flight
*The Kalman filtering fusion of the GPS data and the IMU data is added. The fusion of GPS and PBVS adopts the method of weighting according to the uncertainty of sensor measurement
*PID control is used in the PTZ control method, and the inputs are the offsets of the target image
When the UAV basically reaches the landing platform, it is only adjusted according to the pixel coordinate
'''
import airsim
import time
import math
import pprint
import os
import tempfile
import numpy as np
import cv2
import cv2.aruco as aruco
import xlwt
import threading
from functions import *

lock = threading.Lock()

wb = xlwt.Workbook(encoding='utf-8')
ws = wb.add_sheet('data')
ws.write(0, 0, label = 'UAV_position_x_val')
ws.write(0, 1, label = 'UAV_position_y_val')
ws.write(0, 2, label = 'UAV_position_z_val')
ws.write(0, 3, label = 'GPS_IMU_x')
ws.write(0, 4, label = 'GPS_IMU_y')
ws.write(0, 5, label = 'PBVS_x')
ws.write(0, 6, label = 'PBVS_y')
ws.write(0, 7, label = 'fusion_x')
ws.write(0, 8, label = 'fusion_y')
row = 1

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

#设定水平最大速度、最大降落速度、安全降落高度
#set the horizontal maximum speed, the maximum landing speed and the safe landing height
vmax = 5
vzmax = 2
safe_height = 3

#起飞，飞到目标点附近某位置
#take off and fly to a position near the target point
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()
client.moveToPositionAsync(-20, 16, -18, 5).join()
time.sleep(10)

#计算相机初始角度，转动相机对准目标点
#calculate the initial angles of the camera, rotate the camera to aim at the target point
camera_yaw, camera_pitch = camera_init()
#print(camera_yaw, camera_pitch)
client.simSetCameraOrientation(0, airsim.to_quaternion(camera_pitch, 0, camera_yaw))
save_images()
offset_x, offset_y, pixel_length, theta = aruco_detection('../Appdata/Local/Temp/airsim_drone/0.png', image_width, image_height)
offset_x_last = 0
offset_y_last = 0

#获取无人机当前位置，设定初始速度
#get current location of the UAV and set the initial speeds
UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
GPS_x = UAV_position_x_val + np.random.randn(1)*0.5
GPS_y = UAV_position_y_val + np.random.randn(1)*0.5
PBVS_x = UAV_position_x_val + np.random.randn(1)*0.2
PBVS_y = UAV_position_y_val + np.random.randn(1)*0.2
GPS_x = float(GPS_x)
GPS_y = float(GPS_y)
PBVS_x = float(PBVS_x)
PBVS_y = float(PBVS_y)
dt = 0.59
GPS_r = 1
GPS_IMU_x_p = 0.5
GPS_IMU_y_p = 0.5
GPS_IMU_q = 0
GPS_IMU_x_K = GPS_IMU_x_p/(GPS_IMU_x_p + GPS_r)
GPS_IMU_y_K = GPS_IMU_y_p/(GPS_IMU_y_p + GPS_r)
GPS_IMU_x = GPS_x
GPS_IMU_y = GPS_y
PBVS_r = 0.4
PBVS_IMU_x_p = 0.2
PBVS_IMU_y_p = 0.2
PBVS_IMU_q = 0
PBVS_IMU_x_K = PBVS_IMU_x_p/(PBVS_IMU_x_p + PBVS_r)
PBVS_IMU_y_K = PBVS_IMU_y_p/(PBVS_IMU_y_p + PBVS_r)
PBVS_IMU_x = PBVS_x
PBVS_IMU_y = PBVS_y
fusion_x = PBVS_r*PBVS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*GPS_IMU_x + GPS_r*GPS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*PBVS_x
fusion_y = PBVS_r*PBVS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*GPS_IMU_y + GPS_r*GPS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*PBVS_y
UAV_vx, UAV_vy, UAV_vz = speed_calculation(UAV_position_x_val, UAV_position_y_val, UAV_position_z_val, safe_height, vmax, vzmax)

#线程一：数据处理
#Thread1:Data processing
def data_processing():
    global lock
    global row
    global camera_pitch, camera_yaw
    global offset_x, offset_y, offset_x_last, offset_y_last
    global pixel_length
    global theta
    global UAV_position_x_val, UAV_position_y_val, UAV_position_z_val
    global dt
    global GPS_x, GPS_y, PBVS_x, PBVS_y
    global GPS_IMU_x, GPS_IMU_y, PBVS_IMU_x, PBVS_IMU_y
    global GPS_IMU_x_K, GPS_IMU_y_K, PBVS_IMU_x_K, PBVS_IMU_y_K
    global GPS_r, GPS_IMU_x_p, GPS_IMU_y_p, GPS_IMU_q
    global PBVS_r, PBVS_IMU_x_p, PBVS_IMU_y_p, PBVS_IMU_q
    global fusion_x, fusion_y
    global camera_height
    global UAV_vx, UAV_vy, UAV_vz
    global safe_height
    global vmax, vzmax
    while UAV_position_z_val > safe_height:
        #保存相机图像
		#save the camera images
        save_images()
		
        lock.acquire()
		#计算偏移量
		#calculate the target coordinate offsets
        offset_x_last = offset_x
        offset_y_last = offset_y
        if abs(fusion_x) > 2 or abs(fusion_y) > 2:
            offset_x, offset_y, pixel_length, theta = aruco_detection('../Appdata/Local/Temp/airsim_drone/0.png', image_width, image_height)
        else:
            offset_x, offset_y, pixel_length, theta = aruco_detection('../Appdata/Local/Temp/airsim_drone/1.png', image_width, image_height)
        print(offset_x, offset_y)
		
        UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
        camera_height = -client.simGetCameraInfo(0).pose.position.z_val
        
        if abs(fusion_x) > 2 or abs(fusion_y) > 2:
		    #GPS+IMU
            UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
            UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
            print(UAV_position_x_val, UAV_position_y_val, UAV_position_z_val)
            GPS_x = UAV_position_x_val + np.random.randn(1)*0.5
            GPS_y = UAV_position_y_val + np.random.randn(1)*0.5
            GPS_x = float(GPS_x)
            GPS_y = float(GPS_y)
            #print(GPS_x, GPS_y)
            GPS_IMU_x, GPS_IMU_x_K, GPS_IMU_x_p = kalman_filter_update(GPS_IMU_x, UAV_vx/5, dt, GPS_x, GPS_IMU_x_K, GPS_r, GPS_IMU_x_p, GPS_IMU_q)
            GPS_IMU_y, GPS_IMU_y_K, GPS_IMU_y_p = kalman_filter_update(GPS_IMU_y, UAV_vy/5, dt, GPS_y, GPS_IMU_y_K, GPS_r, GPS_IMU_y_p, GPS_IMU_q)
            #print(GPS_IMU_x, GPS_IMU_y)
		
		    #PBVS
            PBVS_x, PBVS_y = relative_position_solution(camera_height, offset_x, offset_y, pixel_length, theta)
            print(PBVS_x, PBVS_y)
			
			#相对位置信息融合
			#relative position information fusion
            fusion_x = PBVS_r*PBVS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*GPS_IMU_x + GPS_r*GPS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*PBVS_x
            fusion_y = PBVS_r*PBVS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*GPS_IMU_y + GPS_r*GPS_r/(GPS_r*GPS_r + PBVS_r*PBVS_r)*PBVS_y
            print(fusion_x, fusion_y)
	    
        #无人机速度计算和更新
		#calculate and update the UAV speeds
        if abs(fusion_x) < 2 and abs(fusion_y) < 2:
            UAV_vx = -offset_y*0.01
            UAV_vy = offset_x*0.01
            UAV_vz = vzmax
        print(UAV_vx, UAV_vy, UAV_vz)
		
        ws.write(row, 0, label = UAV_position_x_val)
        ws.write(row, 1, label = UAV_position_y_val)
        ws.write(row, 2, label = UAV_position_z_val)
        ws.write(row, 3, label = GPS_IMU_x)
        ws.write(row, 4, label = GPS_IMU_y)
        ws.write(row, 5, label = PBVS_x)
        ws.write(row, 6, label = PBVS_y)
        ws.write(row, 7, label = fusion_x)
        ws.write(row, 8, label = fusion_y)		
        row = row + 1
        
        lock.release()
   

#线程二：云台
#Thread2:PTZ
def PTZ(Kp = 0.0005, Ki = 0.00002, Kd = 0.00003, dt=0.2):
    global lock	
    global offset_x, offset_y, offset_x_last, offset_y_last
    global camera_pitch, camera_yaw
    while abs(fusion_x) > 2 or abs(fusion_y) > 2:
        lock.acquire()
        delta_yaw = Kp*offset_x + Ki*offset_x*dt + Kd*(offset_x - offset_x_last)/dt
        delta_pitch = -(Kp*offset_y + Ki*offset_y*dt + Kd*(offset_y - offset_y_last)/dt)
        camera_yaw = camera_yaw + delta_yaw
        camera_pitch = camera_pitch + delta_pitch	
        print(camera_yaw, camera_pitch)
        client.simSetCameraOrientation(0, airsim.to_quaternion(camera_pitch, 0, camera_yaw))
        lock.release()
        time.sleep(0.2)
	
#线程三：无人机飞行
#Thread3:UAV flight
def fly(): 
    global lock
    global UAV_vx, UAV_vy, UAV_vz
    thread1 = threading.Thread(target=data_processing, args=())
    thread2 = threading.Thread(target=PTZ, args=())	
    thread1.setDaemon(True)
    thread2.setDaemon(True)
    thread1.start()
    thread2.start()
    while UAV_position_z_val > safe_height: 
        lock.acquire()
        client.moveByVelocityAsync(UAV_vx/5, UAV_vy/5, UAV_vz/5, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0))
        lock.release()
        time.sleep(0.1)
    client.landAsync().join()

#降落
#landing
if __name__ == '__main__':
    fly()

wb.save('data.xls')

#断开与Airsim模拟器的链接
#disconnect from Airsim simulator
client.armDisarm(False)
client.enableApiControl(False)
