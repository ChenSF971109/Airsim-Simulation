'''
以监听键盘动作的例程改写的使用键盘控制Airsim无人机的程序
A program that uses the keyboard to control the Airsim drone, rewritten by an example that monitors the keyboard actions
'''
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
client.moveToPositionAsync(0, 0, -3, 1).join()
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
      client.moveByVelocityAsync(2, 0, 0, 0.5).join
    elif event.Key == "Down":
      client.moveByVelocityAsync(-2, 0, 0, 0.5).join
    elif event.Key == "Left":
      client.moveByVelocityAsync(0, -2, 0, 0.5).join
    elif event.Key == "Right":
      client.moveByVelocityAsync(0, 2, 0, 0.5).join
    elif event.Key == "W":
      client.moveByVelocityAsync(0, 0, -2, 0.5).join
    elif event.Key == "S":
      client.moveByVelocityAsync(0, 0, 2, 0.5).join
    elif event.Key == "A":
      client.rotateByYawRateAsync(-10, 0.5).join
    elif event.Key == "D":
      client.rotateByYawRateAsync(10, 0.5).join
    else:
      client.moveByVelocityAsync(0, 0, 0, 0.5).join
    return True


def main():
    hm = PyHook3.HookManager()
    hm.KeyDown = onKeyboardEvent
    hm.HookKeyboard()
    pythoncom.PumpMessages()


if __name__ == "__main__":
    main()

