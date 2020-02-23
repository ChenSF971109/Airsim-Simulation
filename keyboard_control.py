import pythoncom
import PyHook3
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()
print("Moving vehicle to (0, 0, -10) at 5 m/s")
z = -10
client.moveToPositionAsync(0, 0, z, 5).join()
client.hoverAsync().join()
print("""
Manual
"↑":go forward
"↓":go backward
"←":go left
"→":go right
"W":go higher
"S":go lower
"A":turn left
"D":turn right
Ready?
""")


def onKeyboardEvent(event):
    global z
    if event.Key == "Up":
      client.moveByAngleZAsync(-0.5, 0, z, 0, 0.5).join
    elif event.Key == "Down":
      client.moveByAngleZAsync(0.5, 0, z, 0, 0.5).join
    elif event.Key == "Left":
      client.moveByAngleZAsync(0, -0.5, z, 0, 0.5).join  
    elif event.Key == "Right":
      client.moveByAngleZAsync(0, 0.5, z, 0, 0.5).join
    elif event.Key == "W":
      if z >= -20:
        z = z-0.1
        client.moveByVelocityZAsync(0, 0, z, 0.1).join
    elif event.Key == "S":
      if z <= -2:
        z = z+0.1
        client.moveByVelocityZAsync(0, 0, z, 0.1).join
    elif event.Key == "A":
      client.rotateByYawRateAsync(-10, 0.5).join
    elif event.Key == "D":
      client.rotateByYawRateAsync(10, 0.5).join
    else:
      client.moveByAngleZAsync(0, 0, z, 0, 0.5).join
    return True


def main():
    hm = PyHook3.HookManager()
    hm.KeyDown = onKeyboardEvent
    hm.HookKeyboard()
    pythoncom.PumpMessages()


if __name__ == "__main__":
    main()

