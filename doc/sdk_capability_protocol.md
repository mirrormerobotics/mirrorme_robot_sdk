# SDK 多产品能力协商与关节遥测协议

本协议在旧 SDK TCP/UDP 协议上增加显式启用的温度和关节状态扩展。普通订阅仍为 24 字节请求、
32 字节响应；旧 `head=0x0001` 状态 payload 仍为 32 字节。新增内容不启用关节控制，
BPW 的 JointLevelControl 继续拒绝。SDK 原生版本保持 2.0.0，扩展协议独立版本化。

协议常量与编解码的唯一实现为 `mirrorme_robot_sdk/src/internal/capability_protocol.h`，
能力目录结构位于同目录的 `robot_capabilities.h`，两者均为内部实现，不随公开头文件安装。
公开业务数据结构为 `mirrorme_robot_sdk/include/joint_temperatures.h` 与 `joint_states.h`。
扩展字段显式使用小端编码，浮点数采用 IEEE-754 float32；
不依赖编译器结构体对齐。现有普通消息的编码规则保持不变。

## 发现与连接生命周期

客户端先直接订阅所需的状态/运动/关节模式，不通过切换普通模式进行协商。
服务端仅在安装能力目录 provider 后，在有效普通订阅响应的 `reserved[2]`
放入 `0x31504143`，小端字节为 `CAP1`。`reserved[0..1]` 继续用于控制来源反馈。
旧 SDK 继续按普通响应工作；新 SDK 仅在正常订阅已接受、session/mode 匹配且标记完整时发起协商。

```text
普通订阅 24B → 普通响应 32B（CAP1）
能力 Query 24B → 能力响应 32B + TLV 目录
能力 Select 24B + TLV 选择 → 能力响应 32B + 确认目录
确认后：旧 UDP 状态 + 独立的动态关节温度 UDP
```

没有 CAP1 时不能试发未知 mode；历史服务端可能将未知模式视为切换或取消控制。
能力操作与授时共用一个 TCP 读取器，先完整读取 32 字节前缀，再按消息类型消费扩展 body。
普通心跳可穿插；能力握手与自动授时串行。服务端必须串行写入完整前缀及 body，禁止心跳插入 body。

能力处理不更新普通订阅、控制模式、控制命令、连接通知或 SDK 业务版本缓存。
语义拒绝与查询超时只影响扩展能力；异常长度、截断帧等导致无法保持 TCP 消息边界时按传输失败关闭。
重连后清除协商及遥测缓存，不沿用旧连接的 token。

## mode 7 请求

外壳沿用 `SubscribeStateReq` 的 24 字节布局，但只在 `mode=7` 下采用以下含义：

| 字段 | 含义 |
| --- | --- |
| `session_id` | 当前已接受的订阅会话 |
| 两个 UDP port、`rate_hz` | 必须为 0；不会改变订阅目的地址或频率 |
| `version` | 扩展协议版本 1 |
| `request_time_ms` | 请求 ID，客户端用其匹配响应 |
| `reserved[0]` | 1=Query，2=Select |
| `reserved[1]` | 随请求外壳发送的 body 字节数，最多 65536 |

Query body 为空。Select body 是绝对启用集合，允许选择空集合来关闭全部扩展。
同一功能只能选择一个版本；只有支持目录明确列出的功能 ID/版本可以启用。
拒绝不改变上次成功选择。同一集合重复选择保持 token，集合变化分配新 token。

## 能力响应前缀

这是新的 32 字节响应类型，不能当普通 `SubscribeStateResp` 解读。成功 Query/Select
后都附带完整目录与实际 enabled 集合。失败通常不附 body；错误码与请求 ID 仍返回。

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | session_id |
| 2 | 1 | accepted，0/1 |
| 3 | 1 | operation，1/2 |
| 4 | 2 | 协议版本 1 |
| 6 | 1 | mode，固定 7 |
| 7 | 1 | flags，当前为 0 |
| 8 | 4 | request_id |
| 12 | 4 | error_code |
| 16 | 4 | body 字节数 |
| 20 | 8 | stream_token，成功时非 0 |
| 28 | 4 | CAP1 标记 |

error_code：0=成功，1=协议不支持，2=请求非法，3=超出上限，4=功能不支持，
5=拓扑已变化，6=未订阅，7=body 格式非法。标记最低字节为 0x43，避免与历史授时 revision=2 判别冲突。

## TLV 目录

每条记录使用 `type:u16, flags:u16, length:u32, value:length bytes`，连续排列，无对齐填充。
已知类型的 flags 当前必须为 0；未知类型按 length 跳过。未来参数应增加新的 TLV，
不能复用旧类型、字段或功能 ID。客户端保留未知功能 ID/版本，但只自动选择其已实现的功能。

| type | 内容 |
| --- | --- |
| 1 Device | product_model_type:u32、topology_revision:u32、joint_count:u32；必须且唯一 |
| 2 Feature | id:u32、version:u16、flags:u16；每个支持版本一条 |
| 3 Joint | id:u32、type:u8、reserved:u8=0、name_bytes:u16、UTF-8 name；每个关节一条 |
| 4 Selection | id:u32、version:u16、flags:u16=0；目录中表示实际启用项，请求中表示期望项 |
| 5 SelectionContext | topology_revision:u32；Select 请求必须且唯一 |

功能 ID 1/version 1 表示动态关节温度，ID 2/version 1 表示动态关节位置、速度、力矩与时间戳。
SDK 只选择双方都实现的版本，可以仅启用其中一项，也可以同时启用两项。
新增产品无需增加 SDK 的型号 switch：
product_model_type 是保留原值的 u32，关节数、ID、名称、类型以设备目录为准。
关节类型：0=未知，1=旋转关节，2=轮式关节，3=直线关节；未知类型值可以保留。
当前默认物理槽 ID 为 0..N-1、名称为 joint_i；产品 provider 可提供稳定的专属描述。

上限：body 64 KiB，关节 512 个，功能描述 256 个，关节名 64 UTF-8 字节。
重复 ID、重复功能版本、非法 UTF-8、数量不一致、长度越界等必须拒绝，不发布部分目录。
这些资源上限用于限制内存和输入，不能把超过上限的数据截断为可用目录。

## 动态温度 UDP（head=0x00E1）

仅当前连接已成功选择功能 1/version 1 时发送。沿用 8 字节 UDP 外壳，payload 独立编码。
每个数据报最多 1200 字节，最多 72 个关节记录。大 N 分片后必须收齐才原子更新快照。

payload 前缀为 36 字节：

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | 温度协议版本 1 |
| 2 | 2 | flags=0 |
| 4 | 8 | stream_token |
| 12 | 4 | 全快照 sequence |
| 16 | 4 | topology_revision |
| 20 | 8 | 机器人单调时钟采样时间 ms，不是 Unix 时间 |
| 28 | 2 | 本快照总关节数 |
| 30 | 2 | 本分片起始目录索引 |
| 32 | 2 | 本分片记录数 |
| 34 | 2 | reserved=0 |

每条关节记录为 16 字节：`joint_id:u32, valid:u16, reserved:u16=0, motor_celsius:f32,
driver_celsius:f32`。valid bit0/bit1 分别表示电机/驱动器温度有效；未设置时数值槽不表示温度。
有效温度必须有限，0℃和负温度均可有效；不再使用旧包的 +60 编码。

所有分片的 token、sequence、拓扑、时间及总数必须一致，slot 的 ID 必须与目录一致。
旧 sequence、跨会话、跨拓扑、冲突重叠、非法长度不能刷新数据新鲜度。
相同分片重发不产生新样本；乱序分片可组帧。半帧不会暴露给调用者，也不能无限期累积。

SDK 用本机 steady clock 计算 age_ms，完整快照达到 3000ms 即不可用；重连清除缓存。
硬件读数有效性基于专属反馈的接收时间，不以默认值或任何温度数值作为有效性哨兵。

## 动态关节状态 UDP（head=0x00E2）

仅当前连接成功选择功能 2/version 1 时发送，复用普通状态订阅的 UDP 目的地址，不启动
JointLevelControl、不启用关节控制或独立高频控制端口。旧 `head=0x1000` 的 144 字节
12 轴位置/速度/力矩 payload 保持不变。

新 payload 使用与温度协议同布局的 36 字节前缀，wire version=1、flags/reserved=0，
包含 token、sequence、topology_revision、64 位机器人单调时间、总关节数、起始索引与分片记录数。
每条关节记录为 20 字节：

```text
joint_id:u32, valid:u16, reserved:u16=0,
position:f32, velocity:f32, torque:f32
```

valid bit0/1/2 分别表示位置/速度/力矩有效。有效数值必须有限，0 为有效读数；
无效通道保留为 optional/None，不用默认零代替真实反馈。
旋转及轮式关节使用 rad、rad/s、N·m，零偏与方向变换与机器人现有状态接口一致。
每包最多 1200 字节、57 条记录；超过 57 轴分片后完整组帧，支持至 512 轴。
每个完整快照独立递增序号，与温度通道分开管理，token/目录来自同一连接快照。

发送频率受普通状态订阅频率控制，默认 100 Hz、上限 1000 Hz，不因 BPW 禁止旧 12 轴包
而关闭新通道。硬件只在完整反馈到达时记录读数和接收时间，缺失、超过 1000ms 或非有限值
按通道置无效；SDK 从该帧首片到达时计算年龄，完整快照及未完成分片达到 1000ms 即不可用，
末片迟到不能重新延长该帧有效期。
SDK 拒绝错误来源 IP/token/拓扑/ID、旧序号、冲突重叠、非法长度与有效位；
重复包不续期，断连、重连、目标 IP 变化均清缓存。

## C++ / Python 与旧协议回退

能力发现、目录缓存、版本选择与旧协议回退均由 SDK 内部自动完成，不提供公开的能力查询接口，
业务调用方不需要读取目录或手动执行 Query/Select。
公开接口保留 `getJointTemperatures` 的 bool 与 Value 形式，另给两个温度方法增加
`vector<float>&` 重载。快照允许逐轴 optional 温度；vector 重载要求该类温度全部有效，失败不修改输出。
Python `getJointTemperatures()` 返回温度 dict，现有温度方法返回实际 N 项列表，缺少完整数据时返回 None。

关节状态公开 `getJointStates` / `getJointStatesValue`，按关节 ID 返回同一快照的三类读数、
采样时间及年龄。`getJointPosition` / `getJointVelocity` / `getJointTorque` 新增
`vector<float>&` 重载，仅在所请求的维度全部有效时成功，失败不改写输出。
Python 同名方法返回实际 N 轴列表，`getJointStates()` 返回含逐维 None 的详细字典。

C++ `std::array` 的长度在编译期确定，使用 `getJointPositionArray<N>()`、
`getJointVelocityArray<N>()`、`getJointTorqueArray<N>()` 读取恰好 N 轴；轴数不符返回 nullopt，
不截断或补零。三个旧无参 Array 和三个 `float[12]` 关节状态方法标记 `[[deprecated]]`，
仍保持固定 12 轴原语义，BPW 继续拒绝；迁移到 vector 重载、显式 `Array<N>()` 或完整状态快照。
已有 `getJointStateTimestamp(uint32_t*)` / Value 在新通道可用时返回完整快照时间的低 32 位，
完整 64 位时间从 `JointStates.timestamp_ms` 取得；旧 BPX 未广告新关节状态能力时保留旧时间戳语义。

未取得扩展目录时，动态温度仅对明确识别的 BPX/BPX-Pro 回退旧 12 轴协议；
该回退快照 topology_revision=0、sequence=0，采样时间保留旧包的低 32 位相对时间。
BPW、未知或未来产品不能猜测 12 轴。已取得目录但选择被拒绝或超时时，enabled 为空、
动态温度不可用；已经选择新温度协议后，丢包/过期也不能回退旧温度槽。
动态关节状态仅对明确识别的 BPX/BPX-Pro、且目录未广告功能 2 的设备回退旧 12 轴反馈；
这也包括仅支持功能 1 的早期协商设备。只要广告功能 2，选择失败、版本不兼容、
分片不完整或数据过期都不回退旧通道。BPW 和未知产品不能猜测 12 轴。
旧 C++ float[12]/array<float,12> 接口继续保留原有语义，不向调用方写入超过 12 项。
四个固定 12 轴温度接口也标记 `[[deprecated]]`，提示迁移到 `vector<float>&` 重载或
`getJointTemperatures` / `getJointTemperaturesValue`，与六个旧关节状态接口合计十个弃用方法。
动态重载、显式 `Array<N>()`、关节时间戳和 Python 同名动态接口不弃用。

旧 SDK 不会因新增固件而自动获得 N 轴能力；既有 BPW getter 的保护仍保留。
这保证共同支持功能的通信兼容，不代表 bpx_sdk 到 mirrorme_robot_sdk 的 C++ ABI 可以互换。
