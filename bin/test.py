import lvx_binding as lvx
import time

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
data = lidar.get_data(None)
print(data.shape)

