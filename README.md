# EVA系列机型使用手册

## 目录
 * ### 单相机(t265)机型[使用说明](https://github.com/BIT-zhwang/uav-release/blob/master/t265.md)

 * ### t265与d435多相机机型[使用说明](https://github.com/BIT-zhwang/uav-release/blob/master/t265+d435.md)

 * ### [状态机与航点](https://github.com/BIT-zhwang/uav-release/blob/master/状态机.md)

 * ### [建图](https://github.com/BIT-zhwang/uav-release/blob/master/2d建图.md)
 
 * ### [多机通信与坐标统一](https://github.com/BIT-zhwang/uav-release/blob/master/多机通信与坐标统一.md)

说明:
一般的飞行流程为：
1. 启动相机
 - 单相机机型[参考](https://github.com/BIT-zhwang/uav-release/blob/master/t265.md)
 - 双相机机型[参考](https://github.com/BIT-zhwang/uav-release/blob/master/t265+d435.md)
2. 开启建图
 [参考](https://github.com/BIT-zhwang/uav-release/blob/master/2d建图.md)
3. 校正坐标偏差与位姿交换
 [参考](https://github.com/BIT-zhwang/uav-release/blob/master/多机通信与坐标统一.md)
4. 检查状态机，设定各飞机航线
 [参考](https://github.com/BIT-zhwang/uav-release/blob/master/状态机.md)
5. 起飞
 [参考](https://github.com/BIT-zhwang/uav-release/blob/master/状态机.md)

典型的控制命令顺序(以领航机为例):
```
roslaunch realsense2_camera rs_d400_and_t265.launch #启动相机
roslaunch vision_to_mavros t265_go.launch # 连接px4
roslaunch occupancy occupancy_live_rviz.launch # 建图
cd elf # 进入文件夹
python uavx.py # 开启坐标校正
python smatch.py # 开启状态机 
rosrun px4code takeoff # 起飞
```

## 调试与控制

飞机的控制有如下两种方法：
* 配对好的图传
* 局域网远程连接

建议使用局域网下的远程连接方法，**图传设备与无线键鼠存在一定的信号干扰**。
推荐使用NoMachine进行远程连接，飞机内部已安装Nomachine相应版本，可以在[release](https://github.com/BIT-zhwang/uav-release/releases/tag/nomachine_packages)中下载对应平台的安装包。  
在局域网的选择上，2.4GHz信号覆盖范围广，传输速度差；5GHz信号覆盖范围有限，但传输速率高。根据情况选择合适的频率通道。

## 地面站的设置与参数调节

在[release](https://github.com/BIT-zhwang/uav-release/releases/tag/QGroundControl)中下载地面站控制安装包。

## 反馈

当你在使用过程中发现问题或未知的#error#时，欢迎在issues中提问，我们会尽快为您解决相关问题并推送。


