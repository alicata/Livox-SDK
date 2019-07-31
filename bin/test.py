import lvx_binding as lvx
import time

print("import ok")
sum = lvx.add(2,3)
print("sum is ", sum)


print("create Lidar object")
lidar = lvx.Lidar()

print("init lidar")
lidar.init()

print("start lidar")
lidar.start()

print("...")
time.sleep(3)

print("get data")
data = np.zeros((171, 224), dtype=np.double)
data = lidar.get_data(data)


print('stop')
lidar.stop()
lidar.destroy()

