
#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <apr_general.h>
#include <apr_getopt.h>
#include <algorithm>
#include <string.h>
#include "lvx_file.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;



DeviceItem devices[kMaxLidarCount];
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
std::vector<std::string> broadcast_code_rev;
std::condition_variable condition_variable;
std::mutex mtx;
int lvx_file_save_time = 10;
bool is_finish_extrinsic_parameter = false;
bool is_read_extrinsic_from_xml = false;

#define FRAME_RATE 20

/** Connect all the broadcast device in default and connect specific device when use program options or broadcast_code_list is not empty. */
std::vector<std::string> broadcast_code_list = {
  //"000000000000002",
  //"000000000000003",
  //"000000000000004"
};

py::array_t<double> get_array(py::array_t<double> input1) {
    py::buffer_info buf1 = input1.request();

    if (buf1.ndim < 1)
        throw std::runtime_error("invalid dimensions");

    double *ptr = (double *)buf1.ptr;
    size_t h = buf1.shape[0];

    for (size_t i = 0; i < h; i++)
        ptr[i] = 33; 

    return input1;
}


/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    if (handle < broadcast_code_list.size() && is_finish_extrinsic_parameter) {
      std::lock_guard<std::mutex> lock(mtx);
      LvxBasePackDetail packet;
      packet.device_index = handle;

      printf("------------------\n");
      printf("point cloud: id %d\n", data->id);
      printf("point cloud: N %d \n", data_num);
     
      uint8_t* p = data->data;
      //LivoxRawPoint* data = (LivoxRawPoint *)p; 
      //LivoxRawPoint *data = (LivoxRawPoint *)(data->data[0]);



      lvx_file_handler.BasePointsHandle(data, packet);
      lvx_file_handler.CalcExtrinsicPoints(packet);

      point_packet_list.push_back(packet);

      uint32_t max_points = 2;//std::min<uint32_t>(10, data_num);
      for (uint32_t i; i < 2; i++)
      {
          printf("point cloud: p=(%f,%f,%f)\n", packet.point[i].x, packet.point[i].y, packet.point[i].z);
      }

      if (point_packet_list.size() % (50 * broadcast_code_list.size()) == 0) {
        condition_variable.notify_one();
      }
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
}

/** Callback function of get LiDARs' extrinsic parameter. */
void OnGetLidarExtrinsicParameter(uint8_t status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data) {
  if (status == kStatusSuccess) {
    if (response != 0) {
      printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
      std::lock_guard<std::mutex> lock(mtx);
      LvxDeviceInfo lidar_info;
      strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
      memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
      lidar_info.device_index = handle;
      lidar_info.device_type = devices[handle].info.type;
      lidar_info.pitch = response->pitch;
      lidar_info.roll = response->roll;
      lidar_info.yaw = response->yaw;
      lidar_info.x = static_cast<float>(response->x / 1000.0);
      lidar_info.y = static_cast<float>(response->y / 1000.0);
      lidar_info.z = static_cast<float>(response->z / 1000.0);
      lvx_file_handler.AddDeviceInfo(lidar_info);
      if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
        is_finish_extrinsic_parameter = true;
        condition_variable.notify_one();
      }
    }
  }
  else if (status == kStatusTimeout) {
    printf("GetLidarExtrinsicParameter timeout! \n");
  }
}

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
void LidarGetExtrinsicFromXml(uint8_t handle) {
  LvxDeviceInfo lidar_info;
  ParseExtrinsicXml(devices[handle], lidar_info);
  lvx_file_handler.AddDeviceInfo(lidar_info);
  if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
    is_finish_extrinsic_parameter = true;
    condition_variable.notify_one();
  }
}


/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(uint8_t status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of changing of device state. */
void OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }
  printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, OnDeviceInformation, nullptr);
    if (devices[handle].device_state == kDeviceStateDisconnect) {
      devices[handle].device_state = kDeviceStateConnect;
      devices[handle].info = *info;
    }
  } else if (type == kEventDisconnect) {
    devices[handle].device_state = kDeviceStateDisconnect;
  } else if (type == kEventStateChange) {
    devices[handle].info = *info;
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device State error_code %d\n", devices[handle].info.status.status_code);
    printf("Device State working state %d\n", devices[handle].info.state);
    printf("Device feature %d\n", devices[handle].info.feature);
    if (devices[handle].info.state == kLidarStateNormal) {
      if (devices[handle].info.type != kDeviceTypeHub) {
        if (!is_read_extrinsic_from_xml) {
          LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
        }
        else {
          LidarGetExtrinsicFromXml(handle);
        }
        LidarStartSampling(handle, OnSampleCallback, nullptr);
        devices[handle].device_state = kDeviceStateSampling;
      }
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (broadcast_code_list.size() > 0) {
    bool found = false;
    uint8_t i = 0;
    for (i = 0; i < broadcast_code_list.size(); ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i].c_str(), kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  }
  else {
    if ((broadcast_code_rev.size() == 0) || (std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info->broadcast_code) == broadcast_code_rev.end()))
      broadcast_code_rev.push_back(info->broadcast_code);
    return;
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess) {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, nullptr);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}


void app_sleep() {
  #ifdef WIN32
    Sleep(2000);
  #else
    sleep(2);
  #endif
}

class Lidar {
    public:
      Lidar() {}
      bool init();
      bool destroy();
      bool start();
      bool stop();
      py::array_t<double> get_data(py::array_t<double>);
     // uint* get_data() {return nullptr;}; 
};

bool Lidar::init() {
    printf("Livox SDK initializing...\n");
    bool res = Init()==true;

    if (res==true) {
        printf("OK.\n");
        LivoxSdkVersion _sdkversion;
        GetLivoxSdkVersion(&_sdkversion);
        printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

        memset(devices, 0, sizeof(devices));
        SetBroadcastCallback(OnDeviceBroadcast);
        SetDeviceStateUpdateCallback(OnDeviceChange);

    } else {
        printf("Error!\n");
    }
    return res;
}

bool Lidar::start() {
    if (!Start()) 
    {
      return false;
    }
    printf("Started discovering device\n");
    app_sleep();

    if (broadcast_code_rev.size() != 0)
      broadcast_code_list = broadcast_code_rev;

    {
      std::unique_lock<std::mutex> lock(mtx);
      condition_variable.wait(lock);
    }
}

bool Lidar::stop() {
    int i;
    for (i = 0; i < kMaxLidarCount; ++i) {
      if (devices[i].device_state == kDeviceStateSampling) {
  /** Stop the sampling of Livox LiDAR. */
        LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
      }
    }
    return true;
}

int add(int i, int j) {
    return i + j;
}

bool Lidar::destroy() {
    Uninit();
    return true;
}

py::array_t<double> Lidar::get_data(py::array_t<double> input)
{
    size_t N = point_packet_list.size();
    printf("num points: %d\n", (int)N);
    int PACK_POINT_NUM = 100;
    for (LvxBasePackDetail packet : point_packet_list) {
        for (int j; j < PACK_POINT_NUM; j++) {
	    float x = packet.point[j].x;
	    float y = packet.point[j].y;
	    float z = packet.point[j].z;
            printf("p=(%f,%f,%f)\n", x, y, z);
	}
    }

    /*
    for (int i; i < N; i++)
    {
	LvxBasePackDetail packet;
	    
        //packet = point_packet_list.pop_back();
	//float x = p.point[0].x;

        printf("%d x=%lf\n", (int)N, x);

    } 
    */
    return input;
}
 
PYBIND11_MODULE(lvx_binding, m) {
    // optional module docstring
    m.doc() = "lidar plugin";

    m.def("add", &add, "A function which adds two numbers");
    m.def("get_array", &get_array, "test numpy array");

    py::class_<Lidar>(m, "Lidar")
        .def(py::init())
        .def("start", &Lidar::start)
        .def("stop", &Lidar::stop)
        .def("init", &Lidar::init)
	.def("get_data", &Lidar::get_data)
        .def("destroy", &Lidar::destroy);
}

