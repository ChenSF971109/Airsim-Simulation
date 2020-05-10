'''
实现无人机起飞和降落、读取各传感器数据以及保存相机拍摄图像
This program realizes UAV take-off and landing, reading the data of each sensor and saving camera images
'''
import airsim
import time
import pprint
import os
import tempfile
import numpy as np

#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#读取各传感器的数据
#reading the data of each sensor
state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)
imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)
barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)
magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)
gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)
state = client.simGetCameraInfo(3)
s = pprint.pformat(state)
print("state: %s" % s)

landed = client.getMultirotorState().landed_state

airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()

time.sleep(5)
print("Move vehicle to (-10, 10, -10) at 5 m/s")
client.moveToPositionAsync(-10, 10, -10, 5).join()

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)
imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)
barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)
magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)
gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

#保存相机图片
#saving camera images
airsim.wait_key('Press any key to take images')
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

airsim.wait_key('Press any key to land')
client.landAsync().join()
print("already landed...")

client.armDisarm(False)
client.enableApiControl(False)

