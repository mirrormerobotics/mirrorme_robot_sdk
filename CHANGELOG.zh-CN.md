# 更新日志

[English](CHANGELOG.md) | **中文**

本项目所有值得关注的变更都会记录在此文件中。

本文档格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
本项目遵循[语义化版本](https://semver.org/lang/zh-CN/spec/v2.0.0.html)。

## [未发布]

## [2.0.0] - 2026-11-04

### 新增

- 同步已有 BPX-Pro 慢跑步态 `setTrotRun()` / `MotionGait.TrotRun = 15` 的 Python 绑定与示例。
- 支持 BPW 基础状态订阅及运动层：站立、坐下、阻尼、速度、复用 `setWalk()` 的平地，以及
  `setRough()` / `MotionGait.Rough = 16` 的越障步态。
- 同步 Python 绑定、类型存根、C++/Python 示例及不依赖真机的回环测试；保持 SDK 2.0.0 与现有线上报文布局。
- BPW 明确拒绝固定 12 轴关节/温度/腿部里程计读取及关节层控制，不返回截断关节数据或发送不支持命令。

- 补齐 C++/Python 状态查询示例的四组 IMU、全部运动反馈、完整 N 轴列表、独立温度有效性、
  全部故障记录与六组报文时间戳，保留按 ID 的动态快照。
- 使用真实示例子进程与 BPX/BPW/未来产品回环反馈验证非首轴、传感器缺项、观察到的重连和 SIGTERM 正常退出。
- 通过 `getJointStates()` / Value、C++ vector 和显式 `Array<N>`、Python 动态列表
  读取 N 轴位置/速度/力矩，保留关节 ID、逐维有效性、64 位时间戳、完整快照与 1000 ms 新鲜度。
- 使用不依赖硬件的对端验证 16 轴 BPW、未来产品快照、过期或无效数据拒绝、重连与旧 BPX 回退。
- 新增按关节 ID 的 C++/Python `getJointTemperatures()`，用于读取动态关节温度反馈。
- 动态读取 N 轴电机/驱动器温度，不截断轴数，保留单个传感器缺项和采样年龄，拒绝过期或不匹配反馈。
- 明确 BPX/BPX-Pro 的旧 12 轴回退及固定 C++ 接口限制，增加不依赖真机的兼容性、重连和温度测试。

### 变更

- 基于 BPX SDK 的完整历史准备 MirrorMe Robot SDK 2.0.0，统一服务内部开发和
  对外集成。
- pip 分发包改名为 `mirrorme-robot-sdk`；Python 导入、C++ 命名空间、原生库、
  头文件、构建选项及 CI 产物统一使用 `mirrorme_robot_sdk` / `MIRRORME_ROBOT_SDK_*`。
  C++ 应用需重新编译；现有线上报文格式和枚举值保持一致。
- 分发文档与示例的普通状态频率范围同步为原生实现支持的 1 至 1000 Hz；
  旧固件仍受其自身上限限制。
- 补充机型支持范围、固定 12 关节接口、源码/构建边界与旧包迁移说明；
  移除旧预编译产物，新库和 wheel 由改名后的实现生成。
- MSVC 示例和 Python 绑定显式使用 UTF-8 源码编码。
- 更新 master 分支示例，同步中英文构建说明。

- 示例命令由 `gait running` 更名为 `gait run`，不保留旧命令别名；仍调用
  `setRunning()` / `MotionGait.Running = 8`，`trotrun` 仍为 `TrotRun = 15`，BPW 仍不支持 `run`。
  使用新命令须更新并重启 server，SDK API 与枚举不变。
- 统一 C++/Python 运动示例：等待连接完成、识别 BPX/BPX-Pro/BPW 并收到运动反馈，准备 1 秒，
  起立、零速确认 Walk、以 1.0 rad/s 偏航 3 秒、零速 2 秒，最后确认坐下。
- 起立/Walk/坐下阶段均等待新运动时间戳（各最多 10 秒）；连接丢失、反馈停更 1 秒或命令失败时退出，
  支持 Ctrl+C/POSIX SIGTERM 停止。保持 50 Hz 指令、20 ms 主循环及 200 ms 反馈输出；
  仅在运动阶段清理补零，坐下后清理仅断开。
- 状态查询示例每 200 ms 观察连接变化，在连接/重连时显示身份与缓存版本，断连时不展示普通缓存。
- 明确本机展示时间、机器人相对报文/快照时间和 epoch 故障发生时间，移除原有仅 12 轴温度摘要。
- Python 关节位置/速度/力矩 getter 在对应维完整时返回实际 N 轴列表，关节时间戳返回
  新鲜快照低 32 位；旧 C++ 固定 12 轴 ABI 与 BPW 关节控制限制保留。
- 状态示例读取 BPW 动态关节，server 状态优先从新鲜关节快照取得轴数，其次回退温度快照。
- Python 电机/驱动器温度 getter 返回完整动态列表或 `None`；状态示例展示关节 ID/数量，
  server 状态显示可用温度快照的关节数量。
- 新 API 必须使用匹配原生库；Windows/macOS 预编译库须在打包新绑定前重建，SDK 仍为 2.0.0。

### 已弃用

- 将六个旧 C++ 固定 12 轴关节状态 getter 标记为已弃用：
  `getJointPosition(float[12])`、`getJointVelocity(float[12])`、
  `getJointTorque(float[12])`、`getJointPositionArray()`、`getJointVelocityArray()`、
  `getJointTorqueArray()`。保留原有行为；指针方法迁移到同名 `std::vector<float>&`
  重载，无参 Array 方法迁移到对应的显式 `Array<N>()` 模板，或使用
  `getJointStates()` / `getJointStatesValue()`。
- 四个旧 C++ 固定 12 轴温度方法继续标记为已弃用：
  `getMotorTemperature(float[12])`、`getDriverTemperature(float[12])`、
  `getMotorTemperatureArray()`、`getDriverTemperatureArray()`。
  保留原有行为；请迁移到 `std::vector<float>&` 重载或
  `getJointTemperatures()` / `getJointTemperaturesValue()`。
- 合计十个旧状态/温度读取接口已弃用。vector 重载、显式 `Array<N>`、
  `getJointStates()` / `getJointStatesValue()`、关节时间戳 getter 和 Python 动态接口
  不弃用。关节层固定 12 轴 `HighRate` 接口本轮未变更。

### 修复

- 两种运动示例均在 BPW 上跳过零位准备，移除虚假的左侧翻阶段日志；演示使用 Walk，不增加 Rough 阶段。
- C++ 状态示例在 BPW 上跳过固定 12 轴温度读取，避免重复输出不支持提示。

## [1.0.11] - 2026-10-13

### 新增

- C++、Python 新增 `setTumbler()`（含 Python 类型提示）及 `MotionGait.Tumbler = 11`，
  用于 BPX/BPX-Pro 在躺倒（LyingDown）状态下请求倒地爬起；每次调用发起一次恢复请求，
  周期重发不会重复触发。
- C++、Python server/client 示例新增 `tumbler` 命令，C++ server 状态行显示
  Tumbler 步态名。
- 新增回环测试，覆盖 tumbler 指令编码和 C++/Python 全部 server/client 组合。
- Linux、Windows、macOS 新增 CPython 3.15 wheels。CPython 3.8 wheels 通过独立的
  cibuildwheel 3.4.1 旧工具链继续提供（cibuildwheel 4.x 已放弃 Python 3.8）。

### 移除

- 移除未实现的 Python 占位接口 `setUpright()` 及其类型提示；该方法此前始终
  抛出 `NotImplementedError`。

### 修复

- 修复跳跃请求残留导致的重复触发：执行向前跳等跳跃动作后，下一次发送速度
  指令会再次触发同一跳跃动作；跳跃请求现已在动作后清除，速度指令不会再
  重复触发跳跃。
- Windows wheel 构建关闭默认的 delvewheel 修复步骤：wheel 已通过
  `bpx_sdk/lib` 内置 `bpx_sdk_x86_64.dll` 并由 `os.add_dll_directory` 加载，
  cibuildwheel 4.x 默认启用的 delvewheel 找不到该 DLL，会导致构建失败。

## [1.0.10] - 2026-09-11

### 新增

- C++、Python 新增 `setChargingSitDown()`，同步 Python 类型提示和用于运动状态反馈的
  `MotionState.ChargingSitDown = 24` 枚举。
- C++、Python server/client 示例新增 `charging-sit` 命令，状态查询及遥测显示当前和上一运动状态。
- 增加 Python 指令编码、状态反馈和 C++/Python 全部 server/client 组合的回环测试。
- 为 BPX/BPX-Pro 在 C++ 和 Python 中新增原地模式接口 `setPoseTracking()`
  和 `setPose(x, y, z, r)`，并补充 Python 类型提示。姿态输入为 `[-1, 1]`
  范围内的有限归一化值，`r` 对应 yaw。
- 新增使用公共连接参数的 C++ 和 Python 原地模式示例：自动站立并等待运动状态及
  步态反馈，依次执行 `x → y → z → r`，完成后自动趴下并等待完成反馈。
  每轴先在 `-1000` 保持 1 秒，再用 3 秒渐变至 `1000`，保持 1 秒后归零，
  归零后等待 1 秒；输出包含全部四轴数值。
- 新增回环测试，覆盖姿态输入校验、坐标轴行为、步态切换，以及示例动作顺序和时序。
- C++ 状态查询新增 `getHardEstop()` / `getHardEstopValue()`，Python 各状态类新增
  `getHardEstop()`（含类型提示）：硬急停开关按下（所有关节掉电）时返回 1，释放
  返回 0，1 Hz 反馈到达前返回 `None`/false。该状态由 1 Hz 状态标志位 bit 2 携带，
  需要机器人固件支持发布。
- C++ 状态查询新增 `getFaultState()` / `getFaultStateValue()`，全部 Python
  状态类新增 `getFaultState()`（含 `FaultEntry` / `FaultState` 类型提示）：
  返回专用 1 Hz UDP 通道（head `0x00F0`）的全量活动故障快照，最多 32 条
  `{code, occurred_t_ms}` 条目及 `truncated` 标志。条目按 severity 降序排列，
  `entries[0]` 即当前最高优先级故障；机器人无故障时仍以 `entry_count=0`
  心跳包持续上报。返回 `None`/false 仅表示尚未收到故障包（例如旧固件）。
- 新增 `getFaultTimestamp()` / `getFaultTimestampValue()`：最近一帧故障快照
  的接收时间（单调时钟，与 Unix epoch 毫秒的 `occurred_t_ms` 不可互减）。
- 新增公共头 `fault_code.h`：64 位故障码位域说明（version / separator /
  severity / module / sub-module / sub-code）、`DecodeFault*()` 解码辅助与
  `FaultSeverity` / `FaultModule` 枚举，供应用解码和展示故障码。
- C++ 与 Python 反馈示例增加活动故障摘要输出
  （`faults=<count>[+truncated] top=0x…`）。

### 变更

- 原生 SDK 头文件及动态库更新至 1.0.10。
- 更新公开头文件及配套 Linux x86_64/aarch64、Windows x86_64 和 macOS arm64
  SDK 库，以支持原地模式。
- 补充姿态输入范围、步态反馈要求，以及退出原地模式和断线时的归零行为说明。
- 未知 UDP 包头（head）的 stderr 日志改为按 head 值去重，每个值只打印一次，
  后续新增通道不再刷屏。
- 统一故障快照排序：severity 降序，同严重程度按打包故障码升序排列。

### 兼容性

- 需要发布了故障通道的机器人固件。旧固件不受影响，故障 getter 恒为
  `None`/false。
- 旧版 SDK 配新固件时，stderr 每秒打印一行 `unknown upload head`；升级
  SDK 后噪音消除。

## [1.0.9] - 2026-09-08

### 新增

- 新增 C++/Python server/client 示例，支持五种跳跃命令和完整状态查询，并统一时间戳、电池及充电状态打印。

- 为 BPX 增加向上、向前、向后、向左和向右五种跳跃动作接口。
- 增加整机 SN 和机型查询接口，以及 `RobotModel` 机型枚举。
- 增加当前控制模式查询接口和 `ControlMode` 枚举。
- 补充充电器插入状态的 Python 接口及 C++、Python 示例输出。
- 增加上述接口的 Python 绑定和类型提示。
- 连接初始化时同步查询并打印 SN 和机型，自动重连时重新查询。

### 变更

- 查询构建状态及任务详情时遇到临时网络故障自动重试，并保留等待超时和恢复运行提示。
- Linux x86_64 与 aarch64 wheel 改用原生构建机并行构建，进度展示和产物下载同步适配四个任务。
- 将随附的 Linux、Windows 和 macOS SDK 库及头文件更新至 1.0.9。
- wheel 构建脚本支持终端内原地刷新进度，并高亮成功、失败和取消结果；
  输出到日志时保留逐次进度记录。
- 更新中英文接口说明和使用示例。

## [1.0.8] - 2026-07-24

### 新增

- 增加可靠的 TCP 后台自动重连和有上限的退避机制，并通过 C++ 和 Python
  API 的 `isConnected()` 报告实时连接状态。
- 每次确认连接及重连成功后自动同步机器人时间。仅当绝对时间偏差超过10分钟
  时，机器人使用 SDK 时间更新 Linux `CLOCK_REALTIME` 和硬件 RTC。
- 支持在 1 至 200 Hz 范围内配置普通机器人状态上传频率，同时保留各状态组
  自身的原生频率上限。
- 增加机器人状态上传频率测试，以及 C++ 和 Python TCP 重连监测程序。

### 变更

- `connect()` 现在只表示启动后台“期望连接”，不再保证返回时 TCP 已实际连通；
  发送控制指令前应等待 `isConnected()` 返回已连接。
- 每次建立新的运动控制会话时先发送完整零指令；应用必须重新发送控制指令，
  非零目标才可恢复。
- 关节控制不再重放断开前的指令；重连后必须重新发送完整关节帧或零指令。
- 显式断开运动控制时，尽力发送一次目标值全零的运动指令。
- 将随附的 Linux x86_64、aarch64 SDK 库和 CPython 3.8 至 3.14 release
  wheel 更新至 1.0.8。
- 更新随附的 Windows x86_64 SDK DLL 和导入库，并增加适用于 CPython 3.8
  至 3.14 的 1.0.8 Windows AMD64 wheel。
- 更新随附的 macOS arm64 SDK 库，并增加适用于 CPython 3.8 至 3.14 的 
  1.0.8 macOS 11+ arm64 wheel。
- 将运动级 Python 示例的偏航速度调整为 0.2 rad/s，并将演示时间延长至30秒。

### 修复

- 修复机器人静默忽略 `setZeroPositionsFlag()` 零位准备请求的问题。

### 移除

- 移除 C++ 和 Python 显式机器人授时 API 及其示例。应用只需调用
  `connect()`，SDK 会自动执行授时。
- 移除仓库中的 1.0.7 wheel，改为提供 1.0.8 Linux wheel。

## [1.0.7] - 2026-07-22

### 新增

- 增加 Linux 和 Windows 一键 wheel 构建脚本，使用 `uv` 准备可复用的
  Python 3.11 环境并运行 `cibuildwheel`。
- 增加 Windows 构建选项，可在构建期间临时绕过环境变量和系统代理设置。
- 增加可选的 `BPX_SDK_BUILD_HARDWARE_TESTS` CMake 选项，以及交互式真实
  机器人测试，用于验证 Bound 步态在前进、暂停、再次前进的过程中保持选中。
- 增加仓库属性，将文本文件统一规范为 LF，并将预编译库和 wheel 包明确
  标记为二进制文件。

### 变更

- 将 SDK 版本信息以及随附的 Windows 和 Linux 库更新至 1.0.7。
- 更新仓库中的 wheel，覆盖 CPython 3.8 至 3.14、Linux x86_64、Linux
  aarch64 和 Windows AMD64。
- 在中英文文档中说明 Linux 和 Windows wheel 构建流程。
- 明确主步态和子步态取值、有符号子步态的解释方式以及 Walk 起步行为。

### 修复

- 修复随附 SDK 动态库中调用 `setVelocity()` 会导致已设置步态失效的问题。
- 修复随附 SDK 动态库中 `setPronk()` 实际设置为 Bound、`setBound()` 实际
  设置为 Pronk 的接口行为颠倒问题。

### 移除

- 移除仓库中的 1.0.6 wheel。仓库随 1.0.7 提供 Linux 和 Windows wheel；
  macOS wheel 继续由已配置的 CI 构建流程生成，不再存放于仓库的 wheelhouse。

## [1.0.6] - 2026-07-08

### 新增

- 在 C++ API 中增加按需查询机器人构建版本的接口，以及连接后缓存的版本信息。
- 向 Python 暴露缓存的机器人版本信息，并在 C++ 和 Python 示例中输出该信息。
- 在 C++ 和 Python API 中增加结构化腿部里程计，包含机身速度、世界坐标系
  位置、姿态和角速度。
- 增加适用于 Linux、Windows 和 macOS 的 CPython 3.8 至 3.14 wheel。

### 变更

- 更新随附的 SDK 库和示例，以支持机器人版本查询。

### 移除

- 移除 `getCurrentVelocityBody()` 和 `getCurrentVelocityBodyArray()`，并以
  结构化腿部里程计替代旧的仅包含速度的腿部里程计接口。

## 1.0.5 - 2026-07-07

### 新增

- 增加适用于 Linux x86_64/aarch64、Windows AMD64 和 macOS arm64 的
  release wheel 构建，并支持 CPython 3.14。
- 在示例中增加速度、腿部里程计和最大速度输出。

### 变更

- 更新 SDK 1.0.5 随附的库和 wheel。

## [1.0.4] - 2026-07-07

### 新增

- 增加原生 macOS arm64 SDK 库和 Python 包支持。
- 明确打包后的 Python 绑定支持 Python 3.8 及更高版本。

### 变更

- 扩展 CMake 和 Python 打包的架构检测，以支持 macOS arm64。
- 将 macOS 最低部署版本设为 macOS 11.0。

## 1.0.3 - 2026-06-18

### 新增

- 增加 Python 3.8+ 绑定、类型信息，以及状态查询、运动级控制和关节级控制的
  Python 示例。
- 增加 Windows x86_64 DLL 和导入库打包支持。
- 为 C++ 和 Python 示例增加可配置的命令行连接选项。
- 增加独立的英文和简体中文文档。

### 变更

- 使 CMake 示例根据当前平台和架构选择随附的 SDK 库。

### 移除

- 移除不受支持的 `setForwardFlip()` 和 `setBackFlip()` 运动接口。

## 1.0.2 - 2026-05-18

### 新增

- 增加 Walk、Running、翻转、Bipedal、倒立 Bipedal、Pronk、Pace 和 Bound
  的显式步态选择指令。
- 增加子步态反馈读取接口。

## 1.0.1 - 2026-05-07

### 新增

- 增加强类型运动状态、步态和关节索引定义。
- 在原始数值接口之外增加强类型状态和步态读取接口。
- 增加用于机器人零位准备的 `setZeroPositionsFlag()`。

### 变更

- 说明 IMU 欧拉角和四元数的分量顺序。

## 1.0.0 - 2026-04-27

### 新增

- 首次提供用于机器人状态查询、运动级控制和关节级控制的 C++ SDK。
- 首次提供 Linux x86_64 和 aarch64 共享库、CMake 配置及 C++ 示例。

[未发布]: https://github.com/mirrormerobotics/mirrorme_robot_sdk/compare/bca9363...HEAD
[1.0.11]: https://github.com/mirrormerobotics/bpx_sdk_open/compare/v1.0.10...v1.0.11
[1.0.10]: https://github.com/mirrormerobotics/bpx_sdk_open/compare/v1.0.9...v1.0.10
[1.0.9]: https://github.com/mirrormerobotics/bpx_sdk_open/compare/v1.0.8...v1.0.9
[1.0.8]: https://github.com/mirrormerobotics/bpx_sdk_open/compare/v1.0.7...v1.0.8
[1.0.7]: https://github.com/mirrormerobotics/bpx_sdk_open/compare/v1.0.6...v1.0.7
[1.0.6]: https://github.com/mirrormerobotics/bpx_sdk_open/releases/tag/v1.0.6
[1.0.4]: https://github.com/mirrormerobotics/bpx_sdk_open/releases/tag/v1.0.4
[2.0.0]: https://github.com/mirrormerobotics/mirrorme_robot_sdk/compare/bca9363...HEAD
