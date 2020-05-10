'''
相机：0
需与target_tracking.py结合使用
顺序：(1)target_tracking.py(2)auto.py
Camera:0
Need to be used in combination with target_tracking.py
Order:(1)target_tracking.py(2)auto.py
'''
import airsim
import time
import math
import pprint
import os
import tempfile
import numpy as np
import xlwt
import socket

wb = xlwt.Workbook(encoding='utf-8')
ws = wb.add_sheet('data')
ws.write(0, 0, label = 'UAV_position_x_val')
ws.write(0, 1, label = 'UAV_position_y_val')
ws.write(0, 2, label = 'UAV_position_z_val')
ws.write(0, 3, label = 'estimate_x')
ws.write(0, 4, label = 'estimate_y')
ws.write(0, 5, label = 'fusion_x')
ws.write(0, 6, label = 'fusion_y')
ws.write(0, 7, label = 'camera_yaw')
ws.write(0, 8, label = 'camera_pitch')
ws.write(0, 9, label = 'vx')
ws.write(0, 10, label = 'vy')
ws.write(0, 11, label = 'vz')

#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#设定水平最大速度、最大降落速度、安全降落高度
#set the horizontal maximum speed, the maximum landing speed and the safe landing height
vmax = 5
vzmax = 2
h = 3

#起飞，飞到目标点附近某位置
#take off and fly to a position near the target point
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()
client.moveToPositionAsync(-10, 8, -10, 6).join()
time.sleep(5)

#获取无人机当前位置
#get current location of the UAV
UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
if UAV_position_x_val > 0:
    sgn_x = 1
else:
    sgn_x = -1
if UAV_position_y_val > 0:
    sgn_y = 1
else:
    sgn_y = -1

#无人机与相机高度不同，读取相机高度
#the height of the UAV and the camera is different, read the height of the camera
camara_height = -client.simGetCameraInfo(0).pose.position.z_val

#计算相机初始角度，转动相机对准目标点
#calculate the initial angles of the camera, rotate the camera to aim at the target point
x = UAV_position_x_val
y = UAV_position_y_val
z = camara_height
camera_yaw = math.asin(-y/math.sqrt(x*x + y*y))
camera_pitch = math.asin(-z/math.sqrt(x*x + y*y + z*z))
#print(camera_yaw)
print(camera_pitch)
client.simSetCameraOrientation(0, airsim.to_quaternion(camera_pitch, 0, camera_yaw))

#写入初始角度
#write the initial angles
file = open('camera_yaw.txt', 'w')
file.write(str(camera_yaw))
file.close
file = open('camera_pitch.txt', 'w')
file.write(str(camera_pitch))
file.close

file = open('camera_yaw.txt', 'r')
camera_yaw = float(file.read())
file.close
file = open('camera_pitch.txt', 'r')
camera_pitch = float(file.read())
file.close

#计算和比较最小水平移动时间t1和最小降落时间t2，决定降落方案，设定初始速度
#calculate and compare the minimum horizontal moving time t1 and the minimum landing time t2, determine the landing scheme, and set the initial speeds
t1 = math.sqrt(x*x + y*y)/vmax
t2 = UAV_position_z_val/vzmax
if t1 > t2:
    vx = -x/math.sqrt(x*x + y*y)*vmax
    vy = -y/math.sqrt(x*x + y*y)*vmax
    vz = (UAV_position_z_val - h)/t1
else:
    vx = -x/math.sqrt(x*x + y*y)*vmax
    vy = -y/math.sqrt(x*x + y*y)*vmax
    vz = vzmax

#设定执行周期dt
#set the execution cycle dt
dt = 0.2

#GPS信号标志位
#the GPS signal marker
GPS_signal = 1

#GPS信息与视觉信息权重
#the weights of the GPS information and the visual information
GPS_weight = 0.9
visual_weight = 0.1

#PID参数
#the PID parameters
Kp = 1
Ki = 0
Kd = 0

row = 1

#connect to the PTZ tracking system
sk = socket.socket()
sk.connect(('127.0.0.1',8090))

#降落
#landing
while UAV_position_z_val > h:
    client.moveByVelocityAsync(vx, vy, vz, dt).join
    time.sleep(5)
    if abs(UAV_position_x_val) > 0.5 and abs(UAV_position_y_val) > 0.5:
        time.sleep(5)
    else:
	    time.sleep(0.5)

	#更新无人机当前位置
	#update current location of the UAV
    UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
    UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
    UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
    ws.write(row, 0, label = UAV_position_x_val)
    ws.write(row, 1, label = UAV_position_y_val)
    ws.write(row, 2, label = UAV_position_z_val)
	
	#更新相机高度
	#update the camera height
    camara_height = -client.simGetCameraInfo(0).pose.position.z_val
	
	#读取相机角度
	#read the camera angles
    file = open('camera_yaw.txt', 'r')
    camera_yaw = float(file.read().strip())
    file.close
    file = open('camera_pitch.txt', 'r')
    camera_pitch = float(file.read().strip())
    file.close
    print(camera_yaw,camera_pitch)
	
    #推算相对位置,计算期望角度
	#calculate the relative position and the expected angles
    estimate_s = camara_height*math.tan(math.pi/2-abs(camera_pitch))
    estimate_x = -estimate_s*math.cos(camera_yaw)
    estimate_y = -estimate_s*math.sin(camera_yaw)
    #print(UAV_position_x_val,UAV_position_y_val)
    #print(estimate_x,estimate_y)
    estimate_yaw = math.asin(-estimate_y/math.sqrt(estimate_y*estimate_y + camara_height*camara_height))
    estimate_pitch = math.asin(-estimate_x/math.sqrt(estimate_x*estimate_x + estimate_y*estimate_y + camara_height*camara_height))
    ws.write(row, 3, label = estimate_x)
    ws.write(row, 4, label = estimate_y)
	
    #GPS信息与视觉位置信息做融合，更新相机角度
	#the GPS information and the visual position information are integrated to update the camera angles
    x = ((visual_weight*estimate_x) + (GPS_weight*UAV_position_x_val*GPS_signal))/(visual_weight + GPS_weight*GPS_signal)
    y = ((visual_weight*estimate_y) + (GPS_weight*UAV_position_y_val*GPS_signal))/(visual_weight + GPS_weight*GPS_signal)
    z = camara_height
    print(math.sqrt(UAV_position_x_val*UAV_position_x_val + UAV_position_y_val*UAV_position_y_val),UAV_position_x_val,UAV_position_y_val)
    print(estimate_s,estimate_x,estimate_y)
    print(x,y,z)
    ws.write(row, 5, label = x)
    ws.write(row, 6, label = y)
    ws.write(row, 7, label = camera_yaw)
    ws.write(row, 8, label = math.pi/2 + camera_pitch)
	
	#更新无人机速度
	#update the UAV speeds
    t1 = math.sqrt(x*x + y*y)/vmax
    t2 = UAV_position_z_val/vzmax
    if abs(UAV_position_x_val) > 0.5 and abs(UAV_position_y_val) > 0.5:
        vx = -x/math.sqrt(x*x + y*y)*vmax
        vy = -y/math.sqrt(x*x + y*y)*vmax
    else:
        vx = -x/math.sqrt(x*x + y*y)*vmax*(t1/t2)
        vy = -y/math.sqrt(x*x + y*y)*vmax*(t1/t2)
    if abs(UAV_position_x_val) > 6 and abs(UAV_position_x_val) < 9:
        vz = 0
    elif t1 > t2:
        vz = (UAV_position_z_val - h)/t1
    else:
        vz = vzmax
    print(vx,vy,vz)
    ws.write(row, 9, label = vx)
    ws.write(row, 10, label = vy)
    ws.write(row, 11, label = vz)
	
    row = row+1

sk.close()	
		
wb.save('data.xls')	
client.landAsync().join()
time.sleep(5)


client.armDisarm(False)
client.enableApiControl(False)

