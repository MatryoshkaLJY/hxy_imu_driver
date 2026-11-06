HXY IMU ROS 2 驱动
=================

``hxy_imu_driver`` 是 Linux 下的 USB CDC IMU 驱动，读取设备发送的 MAVLink 2
``common/HIGHRES_IMU`` 字节流，发布 ``sensor_msgs/msg/Imu``。包含 ROS 2 节点
``imu_node`` 和可独立使用的命令行接收工具 ``imu_monitor``。

本仓库包含固定版本的官方 MAVLink C 头文件，构建时无需下载协议依赖；不需要
MAVROS、飞控心跳或向设备下发采样命令。已在 ROS 2 Jazzy / Ubuntu 24.04
环境验证。设备固件需提供本文描述的 USB CDC / MAVLink 数据接口。

构建和运行
----------

在已安装 ROS 2 Jazzy 的 Linux 环境中，安装依赖并克隆仓库：

.. code-block:: shell

   source /opt/ros/jazzy/setup.bash
   sudo apt install ros-jazzy-ament-cmake ros-jazzy-rclcpp \
     ros-jazzy-rcl-interfaces ros-jazzy-sensor-msgs ros-jazzy-launch-ros \
     ros-jazzy-ament-cmake-test ros-jazzy-rclpy python3-colcon-common-extensions
   git clone https://github.com/MatryoshkaLJY/hxy_imu_driver.git
   cd hxy_imu_driver
   colcon --log-base build_ros2/log build --base-paths . \
     --build-base build_ros2/build --install-base build_ros2/install \
     --cmake-args -DCMAKE_BUILD_TYPE=Release
   source build_ros2/install/setup.bash
   ros2 launch hxy_imu_driver imu.launch.py

也可将本仓库克隆到现有 ROS 2 工作空间的 ``src/`` 下，从工作空间根目录正常
运行 ``colcon build --packages-select hxy_imu_driver`` 并 source ``install/setup.bash``。

默认自动选择唯一一个产品名以 ``HXY IMU MAVLink`` 开头的设备。多个 IMU 或
使用自定义产品名时指定端口；也支持 ``/dev/serial/by-id/`` 下的稳定设备链接。
驱动会拒绝 SEGGER J-Link 的 CDC 端口，因为它不是 IMU 原生 USB 数据接口。

.. code-block:: shell

   ros2 run hxy_imu_driver imu_node --ros-args -p port:=/dev/ttyACM1
   ros2 topic echo /imu/data_raw --qos-reliability best_effort
   ros2 topic hz /imu/data_raw

修改 ``config/imu.yaml`` 的副本后，可指定自己的参数文件：

.. code-block:: shell

   ros2 launch hxy_imu_driver imu.launch.py params_file:=/absolute/path/to/imu.yaml

默认节点名为 ``hxy_imu``。相对话题 ``imu/data_raw`` 可随 ROS 命名空间变化，
也支持标准话题重映射 ``--ros-args -r imu/data_raw:=/other/topic``。
确认当前用户具有串口读写权限，并避免多个程序同时读取同一串口。

消息约定
--------

默认话题为 ``/imu/data_raw``，消息类型为 ``sensor_msgs/msg/Imu``，使用
``SensorDataQoS``（best effort、volatile、深度 5）。每个通过 CRC、来源和
字段校验的样本发布一次；默认设备固件为 100 Hz。无有效样本时不发布零值。

* 加速度是包含重力的比力，单位 m/s²；角速度单位 rad/s。协议已经使用 SI
  单位，驱动不重复缩放。
* 默认把协议约定的 FRD（前、右、下）转换为 ROS 常用 FLU（前、左、上），
  对加速度和角速度同时应用 ``(x, -y, -z)``。理想 FRD 水平静置输入
  ``(0, 0, -9.80665)`` 会输出 ``(0, 0, +9.80665)``。
* ``header.stamp`` 使用接收时的 ROS ``node.now()``，遵循 ``use_sim_time``。
  MCU 启动微秒数不会直接用作 ROS 时间戳。接收时间含 USB 和调度延迟，
  驱动未实现硬件时间同步。
* 设备没有姿态估计，``orientation_covariance[0] = -1``；单位四元数仅为
  占位。加速度和角速度协方差默认全零，表示未知。
* ``frame_id`` 描述输出数据所属的坐标系；本节点不发布安装位置的 TF。
  磁场、气压、温度在当前固件中为未测量的占位，不发布对应话题。

**安装轴映射仍需通过六面静置和正向旋转测试验证。软件坐标转换不能替代
设备安装轴向和传感器标定验收。**

参数
----

``config/imu.yaml`` 列出全部驱动参数。参数在启动时读取并设为只读，修改后
需要重启节点。

* ``port``：默认 ``""``，自动识别唯一设备；可使用具体串口或 by-id 路径。
* ``frame_id``：默认 ``imu_link``。
* ``topic``：默认 ``imu/data_raw``。
* ``convert_frd_to_flu``：默认 ``true``；``false`` 时保留协议轴，此时应使用
  能准确描述 FRD 数据的 ``frame_id``。
* ``system_id``、``component_id``、``sensor_id``：精确匹配消息来源，默认
  ``1``、``200``、``0``；需与设备固件一致。
* ``reconnect_interval``：重连间隔，默认 ``1.0`` 秒。
* ``data_timeout``：连续没有有效样本超过此时间后重开串口，默认 ``2.0`` 秒。
* ``linear_acceleration_variance``、``angular_velocity_variance``：默认均为
  ``[0.0, 0.0, 0.0]``。分别指定输出坐标系 X/Y/Z 的方差，填入协方差
  对角线；不是标准差。必须为三个有限且非负的数。

断开、读取错误或有效数据超时后，节点关闭串口并按间隔重试。自动识别模式会
重新扫描设备；显式 by-id 路径会重新解析设备链接。每 5 秒日志输出连接状态、
样本数、CRC/字段错误、来源过滤数、重复时间戳和设备时间重置数。

设备协议接口
------------

串口使用 115200、8N1、raw 模式，关闭流控并拉高 DTR。115200 是 USB CDC
名义 line coding，实际 USB 吞吐不由该值决定。设备可在 DTR 拉高后开始发送。

驱动接收 MAVLink 2 ``common/HIGHRES_IMU``（消息 ID 105），默认来源为
``sysid=1``、``compid=200``、消息内 ``id=0``。加速度和角速度必须为有限的
SI 数值，``fields_updated`` 必须为 ``0x003f``。磁场、气压、气压高度、温度
字段必须为零占位值。本驱动针对该六轴设备接口，不是通用 MAVLink 网关。

``time_usec`` 为 MCU 启动后的单调微秒数。相邻有效帧时间戳相同视为重复并
丢弃；时间倒退视为新的启动时间段并计数。串口重连时清空解析器和上一时间戳。
CRC 错误后重新同步，支持 USB 字节流中的拆包和粘包。

不依赖 ROS 的命令行工具
-----------------------

.. code-block:: shell

   cmake -S . -B build_standalone -G Ninja \
     -DIMU_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
   cmake --build build_standalone
   ./build_standalone/imu_monitor --list
   ./build_standalone/imu_monitor --port /dev/ttyACM1 --duration 30 --csv /tmp/imu.csv

不指定端口会自动识别设备；默认持续运行，每秒显示数据和统计，按 Ctrl+C
退出。``--file capture.bin`` 可回放二进制 MAVLink 文件。CSV 记录设备启动
微秒数、来源、序号、传感器 ID 和六轴 SI 数值，保留协议轴，不做 FLU 转换。
命令行工具断开后退出；自动重连由 ROS 2 节点提供。

返回值：0 表示收到有效 IMU 且没有 CRC、字段或时间戳错误；1 表示没有有效帧
或存在上述错误；2 表示参数、端口、设备断开或文件错误。序号缺口和采样频率
只作为统计报告，不单独决定返回值。

测试
----

完成 ROS 2 构建并 source 安装环境后：

.. code-block:: shell

   colcon --log-base build_ros2/log test --base-paths . \
     --build-base build_ros2/build --install-base build_ros2/install \
     --event-handlers console_direct+
   colcon --log-base build_ros2/log test-result \
     --test-result-base build_ros2/build --verbose

独立工具的测试使用 ``ctest --test-dir build_standalone --output-on-failure``。
若不需要测试，构建时可设置 ``-DBUILD_TESTING=OFF``。

测试使用官方 MAVLink 编码器，覆盖序号回绕、拆包/粘包、CRC 错误恢复、
非有限值、来源过滤、重复时间戳、设备时间重置和解析器会话隔离。POSIX
伪终端及真实 ROS 发布/订阅测试检查消息单位、轴向、时间戳、协方差、参数
校验和设备路径重连。测试不需要连接实物 IMU。

Docker
------

主机未安装 ROS 2 时，在仓库根目录使用官方 ROS 2 Jazzy 镜像构建：

.. code-block:: shell

   docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
     -v "$PWD:/driver:rw,z" -w /driver ros:jazzy-ros-base bash -lc \
     'source /opt/ros/jazzy/setup.bash &&
      colcon --log-base build_ros2/log build --base-paths . \
        --build-base build_ros2/build --install-base build_ros2/install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release'

连接设备后启动；以下以 ``/dev/ttyACM1`` 为例，组号从设备文件取得，``:z``
适配 SELinux：

.. code-block:: shell

   docker run --rm -it --user "$(id -u):$(id -g)" \
     --group-add "$(stat -c %g /dev/ttyACM1)" --device /dev/ttyACM1 \
     -e HOME=/tmp -v "$PWD:/driver:rw,z" -w /driver \
     ros:jazzy-ros-base bash -lc \
     'source /opt/ros/jazzy/setup.bash &&
      source build_ros2/install/setup.bash &&
      ros2 launch hxy_imu_driver imu.launch.py'

容器生成的构建产物应在匹配的 ROS 环境中运行。与主机或其他容器的 ROS
节点通信时，可按实际环境增加 ``--network host`` 并使用一致的 ``ROS_DOMAIN_ID``。

实物验证记录
------------

2026-11-06，驱动通过 ROS 2 Jazzy / Ubuntu 24.04 容器中的正式 launch 文件
读取实物 ``/dev/ttyACM1``，独立 ROS 订阅者在约 7 秒观察窗口内收到 675 条
有效消息，按首尾接收时间计算为 100.006 Hz。5 秒状态日志中的 CRC/字段错误、
来源过滤、重复时间戳和设备时间重置计数均为 0，Ctrl+C 后正常退出。
已检查时间戳、``imu_link``、有限六轴值、姿态不可用标记和未知协方差。
这是短时通信验证，未完成安装轴向和传感器标定验收。

许可证与协议来源
----------------

驱动代码使用 Apache-2.0，见 ``LICENSE``。``third_party/mavlink/`` 保留官方
MAVLink C 头文件及其 ``COPYING``，固定提交为
``a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29``；生成代码适用上游的 MIT 输出
例外，来源记录见 ``third_party/mavlink/README.vendor``。

消息语义参考：`官方 Imu 消息定义
<https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg>`_、
`REP-103 坐标和单位约定 <https://www.ros.org/reps/rep-0103.html>`_。
