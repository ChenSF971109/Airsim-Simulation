'''
采集无人机飞行数据，需与fly.py共同使用
Collect the UAV flight datas, need to be used together with fly.py
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

wb = xlwt.Workbook(encoding='utf-8')
ws = wb.add_sheet('data')
ws.write(0, 0, label = 'UAV_position_x_val')
ws.write(0, 1, label = 'UAV_position_y_val')
ws.write(0, 2, label = 'UAV_position_z_val')
ws.write(0, 3, label = 'GPS_eph')
ws.write(0, 4, label = 'GPS_epv')
ws.write(0, 5, label = 'GPS_altitude')
ws.write(0, 6, label = 'GPS_latitude')
ws.write(0, 7, label = 'GPS_longitude')
ws.write(0, 8, label = 'GPS_velocity_x_val')
ws.write(0, 9, label = 'GPS_velocity_y_val')
ws.write(0, 10, label = 'GPS_velocity_z_val')
ws.write(0, 11, label = 'magnetometer_x_val')
ws.write(0, 12, label = 'magnetometer_y_val')
ws.write(0, 13, label = 'magnetometer_z_val')
ws.write(0, 14, label = 'UAV_linear_acceleration_x_val')
ws.write(0, 15, label = 'UAV_linear_acceleration_y_val')
ws.write(0, 16, label = 'UAV_linear_acceleration_z_val')
ws.write(0, 17, label = 'UAV_linear_velocity_x_val')
ws.write(0, 18, label = 'UAV_linear_velocity_y_val')
ws.write(0, 19, label = 'UAV_linear_velocity_z_val')

#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


#获取无人机当前位置
#get current location of the UAV
UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val

row = 1

while UAV_position_z_val > 14:
    UAV_position_x_val = client.getMultirotorState().kinematics_estimated.position.x_val
    UAV_position_y_val = client.getMultirotorState().kinematics_estimated.position.y_val
    UAV_position_z_val = -client.getMultirotorState().kinematics_estimated.position.z_val
    GPS_eph = client.getGpsData().gnss.eph
    GPS_epv = client.getGpsData().gnss.epv
    GPS_altitude = client.getGpsData().gnss.geo_point.altitude
    GPS_latitude = client.getGpsData().gnss.geo_point.latitude
    GPS_longitude = client.getGpsData().gnss.geo_point.longitude
    GPS_velocity_x_val = client.getGpsData().gnss.velocity.x_val
    GPS_velocity_y_val = client.getGpsData().gnss.velocity.y_val
    GPS_velocity_z_val = client.getGpsData().gnss.velocity.z_val
    magnetometer_x_val = client.getMagnetometerData().magnetic_field_body.x_val
    magnetometer_y_val = client.getMagnetometerData().magnetic_field_body.y_val
    magnetometer_z_val = client.getMagnetometerData().magnetic_field_body.z_val
    UAV_linear_acceleration_x_val = client.getImuData().linear_acceleration.x_val
    UAV_linear_acceleration_y_val = client.getImuData().linear_acceleration.y_val
    UAV_linear_acceleration_z_val = client.getImuData().linear_acceleration.z_val
    UAV_linear_velocity_x_val = client.getMultirotorState().kinematics_estimated.linear_velocity.x_val
    UAV_linear_velocity_y_val = client.getMultirotorState().kinematics_estimated.linear_velocity.y_val
    UAV_linear_velocity_z_val = client.getMultirotorState().kinematics_estimated.linear_velocity.z_val
    ws.write(row, 0, label = UAV_position_x_val)
    ws.write(row, 1, label = UAV_position_y_val)
    ws.write(row, 2, label = UAV_position_z_val)
    ws.write(row, 3, label = GPS_eph)
    ws.write(row, 4, label = GPS_epv)
    ws.write(row, 5, label = GPS_altitude)
    ws.write(row, 6, label = GPS_latitude)
    ws.write(row, 7, label = GPS_longitude)
    ws.write(row, 8, label = GPS_velocity_x_val)
    ws.write(row, 9, label = GPS_velocity_y_val)
    ws.write(row, 10, label = GPS_velocity_z_val)
    ws.write(row, 11, label = magnetometer_x_val)
    ws.write(row, 12, label = magnetometer_y_val)
    ws.write(row, 13, label = magnetometer_z_val)
    ws.write(row, 14, label = UAV_linear_acceleration_x_val)
    ws.write(row, 15, label = UAV_linear_acceleration_y_val)
    ws.write(row, 16, label = UAV_linear_acceleration_z_val)
    ws.write(row, 17, label = UAV_linear_velocity_x_val)
    ws.write(row, 18, label = UAV_linear_velocity_y_val)
    ws.write(row, 19, label = UAV_linear_velocity_z_val)
	
    time.sleep(1)
    row = row+1

		
wb.save('ground_truth.xls')	

