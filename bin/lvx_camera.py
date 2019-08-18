import random
import numpy as np
import time
import lvx_binding as lvx

mid100_to_pico = {
    'src_cs' : '-yz-x',
    'minmax' : {
        'x' : [-3.0, +3.0], 
        'y' : [-3.0, +3.0],
        'z' : [0, 5.0]
    },
    'depth_size' : (224, 171),
    'max_samples' : 400000
}

def pcd_to_depth(X, Y, Z, I, sensor_config=mid100_to_pico):
    """Convert point cloud into depth map."""
    depth_width, depth_height = sensor_config['depth_size']
    depth_map = np.zeros((depth_height, depth_width), dtype=np.uint8)
    XYZ = np.stack((X, Y, Z), axis=-1)

    # constrain FOV and distance
    minmax = sensor_config['minmax']
    xmin, xmax = minmax['x']
    ymin, ymax =  minmax['y']
    max_dist =  minmax['z'][1]
    min_intensity = 0

    def in_range(x0, x1, x):
        return x >= x0 and x <= x1

    def clip(x, max_val):
        if x > max_val or x < 0:
            return 0
        return x

    # convert point cloud into depth map
    num_samples = min(len(X), sensor_config['max_samples'])

    # randomly sample from point cloud
    # samp = np.random.choice(np.arange(num_samples), num_samples)
    # X, Y, Z = X[samp], Y[samp], Z[samp]
 
    for i in range(num_samples):
        x, y, z = X[i], Y[i], Z[i]
        if in_range(xmin, xmax, x) and in_range(ymin, ymax, y):
            z = clip(z, max_dist)
            if I[i] > min_intensity:
                px = depth_width - 1 - int(2.0 * depth_width * 0.5 * ((x - xmin) / (xmax - xmin)))
                py = depth_height - 1 - int(2.0 * depth_height * 0.5 * ((y - ymin) / (ymax - ymin)))
                depth_val = int(255 * (z / max_dist))
                depth_map[py, px] = depth_val
                #print('depth:', (px, py, depth_val))
    return depth_map



class Camera:
    def __init__(self):
        print('create Lidar')
        self.lidar = lvx.Lidar()
        print('init() ...')
        self.lidar.init(0)
        print('init completed.')

    def set_device_codes(self, codes):
        self.lidar.set_device_codes(codes)

    def start(self):
        self.lidar.start() 
    
    def stop(self):
        self.lidar.stop() 

    def destroy(self):
        self.lidar.destroy()

    def read(self, timeout=0.200):
        data = np.zeros((171, 224), dtype=np.float64)

        time.sleep(timeout)
        data = self.lidar.get_data(200)
        data = np.reshape(data, (len(data)//4, 4))
        data = data[np.sum(data[:, 0:3], axis=1) > 0.000001]

        #print("data shape", data.shape)
        #print("point 0: ", data[0,:])
        #print("point L: ", data[-1,:])

        X, Y, Z, I = data[:,0], data[:,1], data[:,2], data[:,3]
        #print("Z:", Z[0:10])
        I = I.astype(np.uint8)
        #print("I channel:", I)
        #print(X, Y, Z)
        depth_map = pcd_to_depth(X, Y, Z, I)
        return depth_map, data
