cmake_minimum_required(VERSION 3.0)
# conda activate cxx
# conda install apr boost

set(CMAKE_POSITION_INDIPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)

add_subdirectory(pybind11)
pybind11_add_module(lvx_binding lvx_if.cpp lvx_file.cpp)

set_property(TARGET lvx_binding PROPERTY POSITION_INDIPENDENT_CODE ON) 

target_include_directories(lvx_binding SYSTEM PUBLIC /home/pi/miniconda3/envs/cxx/include/apr-1)
target_link_libraries(lvx_binding PUBLIC /home/pi/miniconda3/envs/cxx/lib/libapr-1.a)
target_link_libraries(lvx_binding PUBLIC /home/pi/miniconda3/envs/cxx/lib/libboost_system.a)
target_include_directories(lvx_binding SYSTEM PUBLIC /home/pi/Livox-SDK/sdk_core/include)
target_link_libraries(lvx_binding PUBLIC /home/pi/Livox-SDK/build/sdk_core/liblivox_sdk_static.a)

install (TARGETS lvx_binding DESTINATION ../../bin) 
    



set(DEMO_NAME lvx_pb11)
add_executable(${DEMO_NAME} main.cpp lvx_file.cpp)
target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )
