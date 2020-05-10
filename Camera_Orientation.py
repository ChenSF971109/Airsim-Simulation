'''
调整相机角度测试程序，相机角度用弧度制表示
Adjusting the camera angles test program, and the camera angles are expressed in radian measure
'''
import airsim
import time
import os
import tempfile
import cv2

#连接到Airsim模拟器
#connect to Airsim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getMultirotorState().landed_state

airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("already flying...")
client.hoverAsync().join()

time.sleep(1)
print("Move vehicle to (-10, 10, -10) at 5 m/s")
client.moveToPositionAsync(0, 0, -3.5, 1).join()

client.simSetCameraOrientation(3, airsim.to_quaternion(-0.5, 0, -0.8))
time.sleep(5)

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
client.moveToPositionAsync(0, 0, -3.5, 1).join()
client.landAsync().join()
print("already landed...")

client.armDisarm(False)
client.enableApiControl(False)