# trai
ADI depth perception with AI

## Config for ADI ToF-3D-Smart-Camera

### How to connect
1. Wi-Fi
- camera has a built-in wifi module accessible, usually  on **ADI_Smart_Camera**
- password: ADI_Smart_Camera
- `ssh -X analog@172.72.1.1`
- password: analog
2. Ethernet
- you can connect an ethernet cable directly to the camera
- connect through Wi-Fi to find the camera IP
- _ssh analog@eth.er.net.ip_
3. Through laptop ethernet
- you can also connect through a ethernet cable from your laptop to camera directly
- first, you need to setup a static IP on the camera, on _ethernet interface_ (e.g. 192.168.1.100)
- set a (manual) static IP on your laptop, with the following configuration:
  * IP: **192.168.1.99**
  * Netmask: **255.255.255.0**
  * Gateway: **192.168.1.1**
- `ssh -X analog@192.168.1.100`

### Get ROS topics
After you've connected to the camera:
- **On camera:**
  - `cd Workspace/aditof_sdk/build`
  - `sudo ./apps/server/aditof-server`
- **On your machine**
  - The following lines can be found (more detailed) here:
    - https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/linux/build_instructions.md
    - https://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/3dsmartcam1/build_instructions.md
    - https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/ros
  - **First time only in a _workspace_ folder:**
    - `cd workspace`
    - **Glog**
    - `git clone --branch v0.3.5 --depth 1 https://github.com/google/glog`
    - `cd glog`
    - `mkdir build_0_3_5 && cd build_0_3_5`
    - `cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..`
    - `sudo make -j4 && sudo make install`
    - `cd ../..`
    - **Libwebsockets**
    - `sudo apt-get install libssl-dev`
    - `git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets`
    - `cd libwebsockets`
    - `mkdir build_3_2_3 && cd build_3_2_3`
    - `cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..`
    - `sudo make -j4 && sudo make install`
    - `cd ../..`
    - **Protobuf**
    - `git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf`
    - `cd protobuf`
    - `mkdir build_3_9_0 && cd build_3_9_0`
    - `cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake`
    - `sudo make -j4 && sudo make install`
    - `cd ../..`
    - **ADITof SDK**
    - `git clone https://github.com/analogdevicesinc/aditof_sdk`
    - `cd aditof_sdk`
    - `mkdir build && cd build`
    - `cmake -DWITH_EXAMPLES=off -DWITH_ROS=on -DUSE_3D_SMART=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..`
    - `sudo cmake --build . --target install`
  - **Usage**
    - I recommend you taking a look here: https://github.com/analogdevicesinc/aditof_sdk/tree/master/bindings/ros#usage 
    - `cd workspace/aditof_sdk/build/catkin_ws`
    - `source devel/setup.bash`
    - `roslaunch aditof_roscpp camera_node.launch ip:="your.cam.era.ip"`
    - **Or as node:** 
    - `roscore`
    - `rosrun aditof_roscpp aditof_camera_node your.cam.era.ip`
  - **After that you should be able to see all the camera topics in any terminal from your machine. Good luck!**
