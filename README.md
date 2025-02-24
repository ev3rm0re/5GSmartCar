# 第十九届全国大学生智能汽车竞赛
## 室外远程驾驶无人车赛
### 重庆第二师范学院鲲鹏队代码仓库

#### 创新点

##### 车道线识别的阈值实时自适应调整

* 自适应阈值控制机制的核心是根据图像中白色像素的数量和线条的数量动态调整二值化阈值，确保图像处理在不同光照和场景条件下的鲁棒性。具体实现上，在Detector中定义adjustThreshold函数通过调整二值化阈值来控制图像的白色像素区域大小，从而提高后续图像处理和目标识别的准确性。

##### 调用神经网络的条件触发机制

* 该机制主要包括斑马线检测触发箭头识别和蓝色/红色区域检测触发字母识别两部分，依次通过传统视觉检测和轻量化神经网络模型协同实现目标识别。

#### 项目配置:

硬件:

* 树莓派4B8G

* 系统: raspios-buster-armhf

依赖项:

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
