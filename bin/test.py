import time
import numpy as np
import cv2

import lvx_camera as lvx

camera = lvx.Camera()
camera.start()

time.sleep(2)
depth_map, data = camera.read()

print('saving depth map...')
cv2.imwrite('depth_map.png', depth_map)
cv2.imwrite('depth_map_30x.png', depth_map*30)
np.savetxt('points.txt', data, delimiter=',')

print('second reading')
depth_map, data = camera.read()
print('data len ', len(data))

