# **【东北大学REAL实验室】自主无人机组装教程** 

Github链接 [REAL_DRONE_400](https://github.com/NEU-REAL/REAL_DRONE_400/) An open source Drone Suite for Aerial Robot.

本文档是视频教程[【东北大学REAL实验室】自主无人机组装教学视频](https://www.bilibili.com/video/BV1hC411H7rh/)的配套文档

实验室主页 [东北大学REAL-LAB 环境感知与自主导航实验室](http://faculty.neu.edu.cn/fangzheng/zh_CN/zdylm/262140/list/index.htm)

  <div align=center>
  <img src="misc/单机图片.jpg"  width="600" align="center">
  </div>

⭐项目亮点⭐

⭐结构件 动力套开源

⭐免雷达驱动一键编译

⭐输出高频雷达定位信息。

<font color="#dd0000">**教程简介**</font>

本次课程是一套面向对自主空中机器人感兴趣的学生、爱好者、相关从业人员的免费课程，包含了从硬件组装、代码部署、实机实验等全套详细流程。提供一种Fast-Drone-250的升级大负载能力、稳定定位版本自主无人机。本次课程所涉及的所有代码、硬件设计全部开源。<font color="#dd0000">严禁商用与转载，版权与最终解释权由东北大学REAL实验室所有。</font>

<font color="#dd0000">**安全事项**</font>

四旋翼无人机具有较高的安全风险，请同学们严格遵守安全规范，不要在有人的室内或室外进行试验，对自己和他人的安全负责，本实验室完全免责。

<font color="#dd0000">**致谢**</font>

感谢飞哥组里的小伙伴们提供的支持以及[FAST-DRONE-250开源项目](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)以及[完整课程](https://www.shenlanxueyuan.com/course/385?source=1)。有兴趣的各位可以去点点⭐。

感谢组内指导老师方正教授的大力支持。

感谢一起调试、设计本构型无人机、提供了大力帮助的好伙伴孙哥和白哥。

# 教程视频链接：
- [第一节：无人机简介与电气连接](https://www.bilibili.com/video/BV1hC411H7rh/?vd_source=a901441d2c723973826f98ab4b1463a5)
- [第二节：工作台与焊接讲解](https://www.bilibili.com/video/BV1zC411H7ij/?vd_source=a901441d2c723973826f98ab4b1463a5)
- [第三节：无人机装配与飞控设置](https://www.bilibili.com/video/BV1Nx4y167wC/?vd_source=a901441d2c723973826f98ab4b1463a5)
- [第四节：PX4_Ctrl FSM讲解与真机定点飞行](https://www.bilibili.com/video/BV11i421y7ta/?vd_source=a901441d2c723973826f98ab4b1463a5)
- [第五节：Preception & Planning与真机飞行](https://www.bilibili.com/video/BV1G1421X7sd/?vd_source=a901441d2c723973826f98ab4b1463a5)

# 项目文件说明：
  项目中分为几个文件夹：
- [files](/files/) 文件夹用于存放各类相关文件
- [home_shfiles](/home_shfiles/) 存放了快速启动脚本，请将此目录下的脚本放置于home路径下以快速启动
- [misc](/misc/) 文件夹存放项目相关的图片资料
- [release](/release/) 文件夹存放项目的硬件设计资料，其中[3MF](/release/3MF/)文件夹存放需要打印的部件模型，我们使用拓竹X1C打印机进行3D打印部分材料制作，耗材使用拓竹PLA-CF。[PRODUCTION](/release/PRODUCTION/)文件夹存放可以直接打印的切片工程。[STEP](/release/STEP/)文件夹存放项目相关的所有硬件设计素材，以STEP文件给出。
- [src](/src/) 文件夹放置项目源码文件。

## 第一章：无人机简介与电气连接
  本次课程与高飞老师的[从零制作自主空中机器人](https://www.bilibili.com/video/BV1WZ4y167me?p=1)同样是一套面向对自主空中机器人感兴趣的学生、爱好者、相关从业人员的免费课程，包含了从硬件组装、机载电脑环境设置、代码部署、实机实验等全套详细流程，带你从0开始，组装属于自己的自主无人机，并让它可以在未知的环境中自由避障穿行。本次课程所涉及的所有代码、硬件设计全部开源，<font color="#dd0000">严禁商用与转载，版权与最终解释权由东北大学REAL实验室所有。</font>

  本次课程的重心主要落在自主空中机器人的搭建、代码部署及调试上，关于自主空中机器人的一些理论基础，例如动力学模型，路径搜索，轨迹规划，地图构建等内容，高飞老师在深蓝学院有非常详尽而深入浅出的[课程](https://www.shenlanxueyuan.com/course/385?source=1)，本次课程就不再赘述。


## 第二章：动力套焊接
  机器人本体相关配件及焊接用工具详见[purchase_list.xlsx](files/purchase_list.xlsx)

  **特别强调，四合一电调的输入电源线需要并联配套电解电容，且输入电源线规格至少应在14AWG以上，推荐12AWG与电池一致，保证过流能力！**

  焊接目标如图
  <div align=center>
  <img src="misc/焊接目标.jpg"  width="600" align="center">
  </div>

  焊接最终结果如图
  <div align=center>
  <img src="misc/焊接结果.jpg"  width="600" align="center">
  </div>

## 第三章：无人机装配与飞控设置

使用QGC连接飞控，如果无法识别为PX4飞控，例如下图：
  <div align=center>
  <img src="misc/飞控烧录.jpg"  width="600" align="center">
  </div>

请按照以下步骤使用dfu-util 工具在ubuntu下刷飞控bootloader：

1.	长按飞控按钮同时插typec连接电脑，按里面txt的命令，在ubuntu下刷引导进去，注意路径
dfu-util -a 0 --dfuse-address 0x08000000 -D ./holybro_kakuteh7mini_bootloader/holybro_kakuteh7mini_bootloader.bin  

2.	进入qgc的固件界面，再重新插拔，自定义安装压缩包里的px4
再重新插拔应该就有了，这个操作可能会报error，不用管他。

    注意：刷1.14 beta版 
    stable版有bug会导致炸机 Dshot不是很稳定会导致怠速过快
    推荐用PWM400输出电调并调整死区至怠速解锁

* 上电前请先用万用表通断档检测电源正负焊点是否短接，强烈建议第一次上电前先接一个[短路保护器](https://item.taobao.com/item.htm?spm=a230r.1.14.6.72b83b20uNbZk7&id=656973651729&ns=1&abbucket=19#detail)

* <font color="#dd0000">检测电机转向前确保没有安装螺旋桨！！！！</font>

* 修改电机转向：调整转向错误电机的3根连接线中的其中两根交换相接即可。


## 第四章：PX4_Ctrl FSM讲解与真机定点飞行
- 自动起飞与定点控制
```
sh start_sensor.sh；
sh start_mapping.sh；
rostopic hz /Odometry；
rostopic hz /localization/validated_odom；
rostopic echo -n 1 /localization/healthy；
```
必须确认 `/Odometry` 持续刷新、`/localization/validated_odom` 持续刷新且
`/localization/healthy` 为 `true`。单看 `/Odom_high_freq` 不够，因为它在没有
激光修正时仍可能由 IMU 传播。拿起飞机进行缓慢的小范围晃动，放回原地后确认
没有太大误差；
遥控器5通道拨到内侧，6通道拨到下侧，油门打到中位；
```
sh start_run_ctrl.sh；
sh start_takeoff.sh;
```
px4ctrl 启动后还会要求健康心跳连续至少 0.5 秒。若起飞命令过早而被拒绝，确认
上述三项和终端错误信息后，再重新发送一次起飞命令。定位故障会锁存；查明原因、
两路里程计恢复且飞机已经落地上锁后，执行：
```
rosservice call /vision_pose_node/reset_fault
```
等待健康重新变为 `true` 后才能再次进入 OFFBOARD。
进入 OFFBOARD 前，px4ctrl 会先发送至少 1 秒定点 setpoint，并等待 PX4 的真实模式
反馈后才允许解锁。飞行中健康心跳、有效里程计、IMU 或 MAVROS 状态任一超时，
px4ctrl 会停止位置控制、请求配置的非 OFFBOARD 模式，并只在最多 0.30 秒内发送
有界的交接 setpoint；随后完全停流，由 PX4 自身的 offboard-loss 策略接管。即使
定位随后恢复，也必须先确认退出 OFFBOARD、落地上锁，再重新发送起飞命令。

试飞时，建议在定位正常后、启动 px4ctrl 和解锁之前另开终端开始记录：

```bash
./home_shfiles/start_flight_record.sh hover_01
```

默认只记录定位、控制、MAVROS/PX4 状态和诊断等关键数据。若需要离线重放
FAST-LIO，再添加 `--with-lidar`；这会显著增加磁盘写入量。落地并上锁后按
`Ctrl+C`，看到保存路径且目录内不再有 `.active` 文件后再断电。日志默认保存在
`~/flight_logs/`，完整选项和故障恢复方法见
`src/realflight_modules/real_drone_bringup/README.md`。

FAST-LIO 的计时诊断默认只在单帧墙钟耗时超过 50 ms，或相邻处理帧起始间隔
超过 250 ms 时输出 `[fastlio_timing]` 告警，并按 1 秒限频。`scope=frame`
会列出 IMU、ICP、地图更新和点云发布等阶段的
`墙钟/主线程 CPU` 耗时；`ikdtree_rebuild_wait_max_ms` 是本帧因 iKD-tree 异步
重建而发生的最长一次搜索等待；`scope=livox_lidar_preprocess` 或
`scope=imu_callback` 则直接指出慢回调。`dominant_hint=parallel_or_wait` 只是说明耗时没有
发生在主线程上，也可能是 OpenMP 工作线程计算，不能单独解释为磁盘 I/O。
`frame_start_gap` 同时给出墙钟间隔和雷达时间戳间隔：只有前者明显大于后者时，
才支持“雷达时间戳连续但进程/回调处理变晚”的判断。阈值可在
`FAST_LIO/config/mid360.yaml` 的 `diagnostics` 段调整，设为 `0` 可关闭相应告警。

标准真机启动脚本使用 `start_realsense_recording.sh`，保留 640 x 480 的 Raw RGB
和 Raw Depth，但将相机图像流统一配置为 15 Hz，以降低 USB、CPU 和录包写盘负载。
如果已有其他 RealSense 节点以不同帧率运行，启动脚本会报错退出，需先关闭旧节点
后重新启动。

如果飞机螺旋桨开始旋转，但无法起飞，说明hover_percent参数过小；如果飞机有明显飞过1米高，再下降的样子，说明hover_percent参数过大；
遥控器此时可以以类似大疆飞机的操作逻辑对无人机进行位置控制；
```
sh start_land.sh
```
降落时把油门打到最低，等无人机降到地上后，把5通道拨到中间，左手杆打到左下角上锁。

## 第五章：Preception & Planning与真机飞行
  确认图中参数：
  <div align=center>
  <img src="misc/Param.png"  width="800" align="center">
  </div>

- Ego-Planner 真机测试

1.启动传感器与定位
```
sh start_sensor.sh；
sh start_mapping.sh；
rostopic hz /Odometry；
rostopic hz /localization/validated_odom；
rostopic echo -n 1 /localization/healthy；
```
2.启动Ego-Planner与Rviz
```
sh start_planner.sh;
sh start_rviz.sh;
```
3.启动PX4_ctrl后确认摇杆正确，自动起飞
```
sh start_run_ctrl.sh；
sh start_takeoff.sh;
```
按下G键加鼠标左键点选目标点使无人机飞行。

### 上桨前 PX4 保护检查

仓库里的 ROS 健康门控不能替代 PX4 自身保护。必须先拆除螺旋桨，在实际使用的
PX4 1.14 固件上确认：外部视觉已被 EKF 融合、水平/垂直位置有效；`ALTCTL` 与
配置的无遥控回退模式确实可切入；以及 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 和
位置丢失动作在停止 OFFBOARD setpoint 后符合预期。仓库所附旧参数文件中的
offboard-loss 动作可能仍依赖位置，未经台架验证不得实飞。

4.任务完成后使用自动降落
```
sh start_land.sh；
```

# 合作与维护
我们会努力扩展所提出的系统并提高代码可靠性。持续维护这个项目

如有任何技术问题，请使用此项目的ISSUE功能。

如有商业咨询，请联系东北大学 方正 教授 (fangzheng@mail.neu.edu.cn)。

# 补充说明：
  如果你觉得这个项目对你能够有所帮助，可以点点⭐或者转发分享哦。
  
  附带几组3D设计图与实物图的对照：
  
  1.整机比对
  <div align=center>
  <img src="misc/REAL_DRONG_400_3D.jpg"  width="600" ">
  <img src="misc/单机图片.jpg"  width="600" ">
  </div>
  2.动力套件
  <div align=center>
  <img src="misc/REAL_DRONG_400_DOWN_PART.jpg"  width="600" ">
  <img src="misc/动力套成品.jpg"  width="600" ">
  </div>
  3.感知模块
  <div align=center>
  <img src="misc/感知模块.jpg"  width="600" ">
  <img src="misc/REAL_DRONG_400_UPPER_PART.jpg"  width="600" ">
  </div>
