# 第十九届全国大学生智能汽车竞赛
## 室外远程驾驶无人车赛
### 重庆第二师范学院鲲鹏队代码仓库

#### 项目配置:

硬件:

* 树莓派4B8G

* 系统: raspios-buster-armhf

软件:

* opencv

* pigpio

* yaml-cpp

项目结构:

```
├── CMakeLists.txt 
├── config/ 
│ └── configs.yaml 
├── include/ 
│ ├── Controller.hpp 
│ ├── Detector.hpp 
│ ├── Logger.hpp 
│ ├── RealTimeVideoCapture.hpp 
│ ├── Struct.hpp 
│ ├── VideoProcessor.hpp 
│ └── VideoRecorder.hpp  
├── models/ 
│ └── arrow-yolo11n-cls-v3.onnx 
├── src/ 
│ ├── Controller.cpp 
│ ├── Detector.cpp
│ ├── VideoProcessor.cpp
│ └── main.cpp
└── README.md 
```