# 一、项目说明
项目时间2025年9月，项目需求是果园场景的自动驾驶。主要需求是在同一个框架下实现两个目标。1、在果园里实现基于先建地图的自动驾驶。2基于牵引信标的自动驾驶，及道路分割。
 
项目框架分为感知、规划、执行三个模块。  

    1、预先建图部分基于RTK的经纬度，输入多个点建立全局地图。手动标注拓扑关系。  
    2、规划部分基于先建地图，输入手持信标的坐标为终点，全局路径规划进行追踪，其中规划部分基于dji算法，实现最短路径。  
    3、执行部分基于现成的bunker底盘，加入基于yolo的行人检测，实现停障的效果

# 二、项目设备和软件列表
## 硬件设备列表：
    工控：小米微型主机、16核cpu、核显
    底盘：松灵bunker_mini  
    相机：奥比中光gemini335L  
    RTK:联适R60/R70  
    IMU：Yesense
## requirements.txt
    Ubuntu 20.04  
    python3  
    miniconda3
    以及若干lib依赖
## YOLO检测的conda环境
    需要安装opencv，opencv-python，numpy，matplotlib，scipy，torch，torchvision，torchsummary，Pillow，scikit-image，scikit-learn
## 道路分割Paddleseg的conda环境
    paddleseg
# 三、项目框架
1Visual_   
# 四、负责人和邮箱
    1、陈佳瑜  负责整体项目技术路线和框架
    2、居人杰 负责信标部分
    3、石垒 负责建图、规划框架和底盘、IMU、RTK
    4、熊昱涵 负责深度相机、YOLO和道路分割部分
