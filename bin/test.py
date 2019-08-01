import lvx_binding as lvx
import time
import numpy as np
import cv2
import lidar_utils as lu

def read_depth():
    time.sleep(0.200)
    data = lidar.get_data(200)
    data = np.reshape(data, (len(data)//4, 4))
    data = data[np.sum(data[:, 0:3], axis=1) > 0.000001]

    print("data shape", data.shape)
    print("point 0: ", data[0,:])
    print("point L: ", data[-1,:])

    X, Y, Z, I = data[:,0], data[:,1], data[:,2], data[:,3]
    #print("Z:", Z[0:10])
    I = I.astype(np.uint8)
    #print("I channel:", I)
    #print(X, Y, Z)
    depth_map = lu.pcd_to_depth(X, Y, Z, I)
    return depth_map, data

lidar = lvx.Lidar()

print("init lidar")
lidar.init()

print("start lidar")
lidar.start()

print("get  data ...")
data = np.zeros((171, 224), dtype=np.float64)

time.sleep(3)

depth_map, data = read_depth()
print('saving depth map...')
cv2.imwrite('depth_map.png', depth_map)
cv2.imwrite('depth_map_30x.png', depth_map*30)
np.savetxt('points.txt', data, delimiter=',')

depth_map, data = read_depth()
print('second reading')

print('data len ', len(data))


print('stop')
lidar.stop()
lidar.destroy()