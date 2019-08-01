import lvx_binding as lvx
import time
import numpy as np

lidar = lvx.Lidar()

print("init lidar")
lidar.init()

print("start lidar")
lidar.start()

print("...")
time.sleep(0.5)

print('stop')
lidar.stop()
lidar.destroy()

print("get data")
#data = np.zeros((171, 224), dtype=np.double)
# reshape to N:3 shape
data = lidar.get_data(None, 200)
data = np.reshape(data, (len(data)//3, 3))
print(data.shape)
print(data)

