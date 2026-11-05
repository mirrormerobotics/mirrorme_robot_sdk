# 机器人 SDK 仓库组织形态调研

调研日期：2026-11-04。范围为厂商官方公开仓库和开发文档；公开 SDK 的交付布局不能用于推断其内部源码仓库如何组织。

**结论：一个主 SDK 仓库支持多款机器人、甚至跨产品线，有明确同行实践。** 常见做法是共享通信和客户端基础设施，再按机型、协议或能力保留差异；语言、ROS、仿真、历史产品也可能有独立仓库。没有证据表明这些公司都将全部产品、全部语言和所有开发工具合并进唯一仓库。

| 厂商 | 官方公开组织形态 | 可确认的支持范围及边界 |
|---|---|---|
| 宇树 Unitree | `unitree_sdk2` 为跨产品的 C++ 主 SDK；Python、ROS2 各有配套仓库；旧协议 SDK 单独保留。 | 官方组织页列出 Go2、B2、H1、G1、H2、R1、A2，覆盖四足与人形；同仓示例另含轮足变体。具体客户端和消息类型仍分机型或协议族。[官方入口](https://github.com/unitreerobotics)、[主仓接口目录](https://github.com/unitreerobotics/unitree_sdk2/tree/main/include/unitree/robot) |
| 智身 Genisom | 官网开源入口指向统一 `genisom_robot_sdk`；采用 C++17、头文件、预编译库和示例。L1 另有 [genisom_L1_sdk](https://github.com/zsibot/genisom_L1_sdk)。 | 主 SDK 明确支持 M1/L2 系列，能力表列出 12 种点足、轮足变体；不能据此宣称 NE01 人形也已接入。[官网入口](https://www.genisomai.com/opensource)、[SDK README](https://github.com/zsibot/genisom_robot_sdk/blob/main/README.md)、[能力表](https://github.com/zsibot/genisom_robot_sdk/blob/main/docs/en/sdk_api_capability_en.md) |
| 云深处 DeepRobotics | 多机型仓库与专用 SDK 并存。`sdk_deploy` 共用 `drdds`，在 `src/<model>_sdk_deploy` 下分包；`deep-robotics-sdk2` 则划分公共层与产品层。 | `sdk_deploy` 列出 Lite3、M20、M20S、S10；SDK2 当前 Products 只列 DR02 Pro/Std。官方文档还提供 X30 `robotserver_sdk`、M20 Android SDK 等，不能把 SDK2 名称理解为已覆盖全部产品。[sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/README.md)、[SDK2](https://github.com/DeepRoboticsLab/deep-robotics-sdk2/blob/main/README.md)、[开发文档](https://developer.deeprobotics.cn/docs/) |
| 逐际动力 LimX | `limxsdk-lowlevel` 同仓交付 C++/Python API、头文件、预编译库、wheel 和示例；机器人模型、训练及仿真在其他仓库。 | README 明确列出 TRON1、TRON2、TRON2 DASF、HU_D04；按机器人类型选择 API。`Wheellegged` 有类型定义，但仓内没有对应例程，不能仅凭枚举认定交付完整。[官方 SDK README](https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/README.md) |

**统一仓库后的接口如何划分**

宇树把公共 `Client`、通信通道与机型客户端放在同一 SDK 内。Go2 使用 `unitree::robot::go2::SportClient`，G1 使用 `unitree::robot::g1::LocoClient`；动作、状态机和 API ID 分别定义。消息结构也分族：`unitree_go` 与 `unitree_hg` 的 `LowCmd_` 电机数组容量分别为 20、35，这些容量不等于实际关节数。[Go2 接口](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)、[G1 接口](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_client.hpp)、[go2 消息](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/LowCmd_.hpp)、[hg 消息](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/hg/LowCmd_.hpp)

智身体现了另一种组织方法：统一 `SDKClient`，握手后获取 `DeviceType`，按设备能力判定操作是否可用；不支持时返回 `Errc::UnsupportedDeviceOperation`。能力表同时提示，机型允许调用不等于相应固件、外设条件已经满足。因此，同一个类可以服务多机型，但必须明确能力和前置条件。[官方能力表](https://github.com/zsibot/genisom_robot_sdk/blob/main/docs/en/sdk_api_capability_en.md)

逐际动力使用公共 `ApiBase` 与具体机器人类型扩展，并提供 `getMotorNumber()`、`getMotorNames()`；`RobotState`、`RobotCmd` 使用 `std::vector` 表示电机数据。这给不同关节数量留出了接口空间。[ApiBase](https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/include/limxsdk/apibase.h)、[数据类型](https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/include/limxsdk/datatypes.h)

**公开 SDK 不一定包含完整核心源码**

宇树 CMake 以 `STATIC IMPORTED` 导入仓库自带的 `libunitree_sdk2.a`；智身交付 `lib/<arch>/librobot_sdk.so`；逐际动力也采用核心预编译库加接口和示例的方式。因此，MirrorMe 当前由内部实现仓库构建原生库，再放入统一 SDK 分发仓库，并不偏离这些公开交付实践。此处比较的是交付物，不能据此断言同行内部也采用同样的双仓流水线。[宇树 CMake](https://github.com/unitreerobotics/unitree_sdk2/blob/main/CMakeLists.txt)、[智身 README](https://github.com/zsibot/genisom_robot_sdk/blob/main/README.md)、[逐际动力 README](https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/README.md)

**对 MirrorMe 当前提交的建议（工程判断）**

当前统一仓库、Python 包及 C++ 命名空间已改为 `mirrorme_robot_sdk`，原生实现仍维护在 `bpx_motion_sdk/c2/mirrorme_robot_sdk`。本地 [`include/motion_types.h`](../../include/motion_types.h) 仍定义 `kJointCount = 12`；[README](../../README.zh-CN.md) 明确 BPX/BPX-Pro 当前可用，BPW 仅可识别，16 关节尚未适配。这些描述应继续作为现阶段的支持边界。

1. **保留公司级仓库和命名。** `mirrorme_robot_sdk`、`mirrorme-robot-sdk`、`MIRRORME_ROBOT_SDK_*` 适合作为跨产品入口；此次调研没有给出改回产品专属名称的理由。
2. **逐步形成公共基础层与机型/能力模块。** 连接、协议会话、错误处理等可以共用；产品特有动作、关节映射、状态结构按实际差异拆分。无需提前创建大量空模块。
3. **接入 BPW 前处理 12/16 关节差异。** 应选择明确的机型类型或动态关节描述，并补齐能力查询、不支持操作的错误语义及支持矩阵；不能只增加一个产品枚举就宣布支持。
4. **语言、协议世代按需要拆分。** C++ 与 Python 暂时同仓可以保留。ROS、仿真或协议不兼容的历史产品将来可独立交付，不必追求所有内容都进入一个仓库。
5. **本次提交继续聚焦命名迁移与完整构建产物。** 目前没有必要仅因同行采用统一 SDK，就立即迁移内部原生源码、修改通信协议或宣称已经具备全产品支持。

资料快照：宇树 SDK2 核对提交为 `8e1b927586712edd6a4c29d50dd52dff1e163439`。上文支持范围以调研时官方公开资料为准；同仓包含某机型目录或类型定义，不代表全部固件、硬件配置和功能均已验证。
