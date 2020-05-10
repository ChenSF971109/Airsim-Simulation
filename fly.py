'''
无人机按照设定的路径飞行，需与data_collect.py共同使用
The UAV flies according to the set path, need to be used together with data_collect.py
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


#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#起飞，飞到目标点附近某位置
#take off and fly to a position near the target point
airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()
client.moveToPositionAsync(-10, -20, -16, 0.5).join()
time.sleep(10)

client.moveToPositionAsync(-10, -15, -15.6, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-12, -14, -16.2, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-8, -12, -17.5, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-10.1, -11, -18.2, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-10.1, -10.1, -18.6, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-10.1, -10.1, -18.6, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-15.3, -10.2, -17.4, 0.5).join()
#time.sleep(10)
client.moveToPositionAsync(-15.3, -20, -15, 0.5).join()
#time.sleep(10)
'''
client.moveToPositionAsync(1, 9, -10, 0.5).join()
client.moveToPositionAsync(2, 8, -10, 0.5).join()
client.moveToPositionAsync(3, 7, -10, 0.5).join()
client.moveToPositionAsync(4, 6, -10, 0.5).join()
client.moveToPositionAsync(5, 5, -10, 0.5).join()
client.moveToPositionAsync(6, 4, -10, 0.5).join()
client.moveToPositionAsync(7, 3, -10, 0.5).join()
client.moveToPositionAsync(8, 2, -10, 0.5).join()
client.moveToPositionAsync(9, 1, -10, 0.5).join()
client.moveToPositionAsync(10, 0, -10, 0.5).join()
client.moveToPositionAsync(9, -1, -10, 0.5).join()
client.moveToPositionAsync(8, -2, -10, 0.5).join()
client.moveToPositionAsync(7, -3, -10, 0.5).join()
client.moveToPositionAsync(6, -4, -10, 0.5).join()
client.moveToPositionAsync(5, -5, -10, 0.5).join()
client.moveToPositionAsync(4, -6, -10, 0.5).join()
client.moveToPositionAsync(3, -7, -10, 0.5).join()
client.moveToPositionAsync(2, -8, -10, 0.5).join()
client.moveToPositionAsync(1, -9, -10, 0.5).join()
client.moveToPositionAsync(0, -10, -10, 0.5).join()
client.moveToPositionAsync(-1, -9, -10, 0.5).join()
client.moveToPositionAsync(-2, -8, -10, 0.5).join()
client.moveToPositionAsync(-3, -7, -10, 0.5).join()
client.moveToPositionAsync(-4, -6, -10, 0.5).join()
client.moveToPositionAsync(-5, -5, -10, 0.5).join()
client.moveToPositionAsync(-6, -4, -10, 0.5).join()
client.moveToPositionAsync(-7, -3, -10, 0.5).join()
client.moveToPositionAsync(-8, -2, -10, 0.5).join()
client.moveToPositionAsync(-9, -1, -10, 0.5).join()
client.moveToPositionAsync(-10, 0, -10, 0.5).join()
client.moveToPositionAsync(-9, 1, -10, 0.5).join()
client.moveToPositionAsync(-8, 2, -10, 0.5).join()
client.moveToPositionAsync(-7, 3, -10, 0.5).join()
client.moveToPositionAsync(-6, 4, -10, 0.5).join()
client.moveToPositionAsync(-5, 5, -10, 0.5).join()
client.moveToPositionAsync(-4, 6, -10, 0.5).join()
client.moveToPositionAsync(-3, 7, -10, 0.5).join()
client.moveToPositionAsync(-2, 8, -10, 0.5).join()
client.moveToPositionAsync(-1, 9, -10, 0.5).join()
client.moveToPositionAsync(0, 10, -10, 0.5).join()
'''
client.moveToPositionAsync(-15, -20, -2, 2).join()
client.landAsync().join()
time.sleep(5)


client.armDisarm(False)
client.enableApiControl(False)
