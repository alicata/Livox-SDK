import time
import numpy as np
import cv2

import lvx_camera as lvx

print("create lvx Camera")
camera = lvx.Camera()

print("set device codes ...")
camera.set_device_codes(["1LVDG1F006T8261", "1LVDG1F006T8262"])

print('START CAMERA ...')
time.sleep(5)
camera.start()

print('read from camera ...')
time.sleep(2)
depth_map, data = camera.read()
depth_map, data = camera.slow_read()

print('saving depth map...')
cv2.imwrite('depth_map.png', depth_map)
cv2.imwrite('depth_map_30x.png', depth_map*30)
np.savetxt('points.txt', data, delimiter=',')

print('second reading')
for i in range(100):
    depth_map, data = camera.read(1)
    cv2.imwrite('depth_map_30x.png', depth_map*30)
    print('data len ', len(data))
    time.sleep(1)

