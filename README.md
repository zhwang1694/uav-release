# EVA系列机型使用手册

 * ## 单相机(t265)机型[使用说明](https://github.com/BIT-zhwang/uav-release/blob/master/t265.md)

 * ## t265与d435多相机机型[使用说明](https://github.com/BIT-zhwang/uav-release/blob/master/t265+d435.md)

 * ## [状态机与航点](https://github.com/BIT-zhwang/uav-release/blob/master/状态机.md)

 * ## [建图](https://github.com/BIT-zhwang/uav-release/blob/master/2d建图.md)
 
 * ## [多机通信与坐标统一](https://github.com/BIT-zhwang/uav-release/blob/master/多机通信与坐标统一.md)

飞机的控制有如下两种方法：
* 配对好的图传
* 局域网远程连接

建议使用局域网下的远程连接方法，**图传设备与无线键鼠存在一定的信号干扰**。
推荐使用NoMachine进行远程连接，飞机内部已安装Nomachine相应版本，可以在[release](https://github.com/BIT-zhwang/uav-release/releases/tag/nomachine_packages)中下载对应平台的安装包。