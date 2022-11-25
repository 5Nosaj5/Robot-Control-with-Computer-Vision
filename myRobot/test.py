import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

def opengz():
    subprocess.run('gz sim ~/.gazebo/models/myRobot/myModel.sdf', shell=True)

def sendVel():
    subprocess.run('gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"')


# # opengz()
#subprocess.call('gnome-terminal')
# #time.sleep(1)
# os.system("gz sim ~/.gazebo/models/myRobot/myModel.sdf")
# #subprocess.call('gz sim ~/.gazebo/models/myRobot/myModel.sdf')

with ThreadPoolExecutor(max_workers=3) as executor:
    future = executor.submit(opengz)
    print('hi')
    time.sleep(5)
    future2 = executor.submit(sendVel)
    #print(future.result())


print('hello')

