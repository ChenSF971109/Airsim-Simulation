'''
相机：3
估计相对位置由无人机相机高度、云台角度和像素点坐标偏移量计算
融合位置信息是由无人机三维坐标(GPS)和估计相对位置简单加权融合得到
相机角度的更新根据融合位置信息来计算
Camera: 3
The estimated relative position is calculated by the height of the UAV camera, PTZ angles and the pixel coordinate offsets
The fusion position information is obtained by simple weighted fusion of the UAV three-dimensional coordinate (GPS) and the estimated relative position
Camera angles update is calculated based on the fusion position information
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
vmax = 10
vzmax = 3
h = 2

#起飞，飞到目标点附近某位置
#take off and fly to a position near the target point
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()
client.moveToPositionAsync(-15, 12, -10, 5).join()
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
camara_height = -client.simGetCameraInfo(3).pose.position.z_val

#计算相机初始角度，转动相机对准目标点
#calculate the initial angles of the camera, rotate the camera to aim at the target point
x = UAV_position_x_val
y = UAV_position_y_val
z = camara_height
camera_yaw = math.asin(-y/math.sqrt(y*y + z*z))
camera_pitch = math.asin(-x/math.sqrt(x*x + y*y + z*z))
#print(camera_yaw)
#print(camera_pitch)
client.simSetCameraOrientation(3, airsim.to_quaternion(camera_pitch, 0, camera_yaw))

#计算和比较最小水平移动时间t1和最小降落时间t2，决定降落方案，设定初始速度
#calculate and compare the minimum horizontal moving time t1 and the minimum landing time t2, determine the landing scheme, and set the initial speeds
t1 = math.sqrt(x*x + y*y)/vmax
t2 = UAV_position_z_val/vzmax
if t1 > t2:
    vx = -x/math.sqrt(x*x + y*y)*vmax
    vy = -y/math.sqrt(x*x + y*y)*vmax
    vz = (UAV_position_z_val - h)/t1
else:
    vx = -x/math.sqrt(x*x + y*y)*(t1/t2)*vmax
    vy = -y/math.sqrt(x*x + y*y)*(t1/t2)*vmax
    vz = vzmax



#设定执行周期dt
#set the execution cycle dt
dt = 0.1

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
	
#降落
#landing
while UAV_position_z_val > h:
    client.moveByVelocityAsync(vx, vy, vz, dt).join
    time.sleep(0.5)
	
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
    img = cv2.imread('../Appdata/Local/Temp/airsim_drone/1.png')
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
    dx = center_x - image_width/2
    dy = center_y - image_height/2

	#更新无人机当前位置
	#update current location of the UAV
    UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
    UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
    UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
	
	#更新相机高度
	#update the camera height
    camara_height = -client.simGetCameraInfo(3).pose.position.z_val
	
    #推算相对位置,计算期望角度
	#calculate the relative position and the expected angles
    estimate_y = -camara_height*math.tan(camera_yaw)*(1 - sgn_y*dy*2/image_height)
    estimate_x = -math.sqrt(estimate_y*estimate_y + camara_height*camara_height)*math.tan(camera_pitch)*(1 + sgn_x*dx*2/image_width)
    #print(UAV_position_x_val,UAV_position_y_val)
    #print(estimate_x,estimate_y)
    estimate_yaw = math.asin(-estimate_y/math.sqrt(estimate_y*estimate_y + camara_height*camara_height))
    estimate_pitch = math.asin(-estimate_x/math.sqrt(estimate_x*estimate_x + estimate_y*estimate_y + camara_height*camara_height))
	
    #GPS信息与视觉位置信息做融合，更新相机角度
	#the GPS information and the visual position information are integrated to update the camera angles
    x = ((visual_weight*estimate_x) + (GPS_weight*UAV_position_x_val*GPS_signal))/(visual_weight + GPS_weight*GPS_signal)
    y = ((visual_weight*estimate_y) + (GPS_weight*UAV_position_y_val*GPS_signal))/(visual_weight + GPS_weight*GPS_signal)
    z = camara_height
    print(UAV_position_x_val,UAV_position_y_val)
    print(estimate_x,estimate_y)
    print(x,y)
    camera_yaw = math.asin(-y/math.sqrt(y*y + z*z))
    camera_pitch = math.asin(-x/math.sqrt(x*x + y*y + z*z))
    client.simSetCameraOrientation(3, airsim.to_quaternion(camera_pitch, 0, camera_yaw))
	
	#更新无人机速度
	#update the UAV speeds
    t1 = math.sqrt(x*x + y*y)/vmax
    t2 = UAV_position_z_val/vzmax
    if t1 > t2:
        vx = -x/math.sqrt(x*x + y*y)*vmax
        vy = -y/math.sqrt(x*x + y*y)*vmax
        vz = (UAV_position_z_val - h)/t1
    else:
        vx = -x/math.sqrt(x*x + y*y)*(t1/t2)*vmax
        vy = -y/math.sqrt(x*x + y*y)*(t1/t2)*vmax
        vz = vzmax
    print(vx,vy)
		
	
client.landAsync().join()
time.sleep(5)


client.armDisarm(False)
client.enableApiControl(False)
