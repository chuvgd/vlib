# vlib 设计文档

> 状态：草案（v0.1），供 review。本文档只描述**目标形态与约定**，不描述现有实现。
> 现有代码的问题清单见 [第 9 节](#9-现状诊断vlib-当前问题清单)。

---

## 1. 定位

vlib 是一个**软硬件无关的中间层工具库**。

一句话概括：

> **只提供积木，不提供框架；但可以用积木搭出框架。**

它对上不规定应用怎么写，对下不规定用谁的 HAL。它只负责把"电机协议、滤波器、PID、底盘运动学"这类**可复用的中间件**做成互相独立、可自由组合的模块。

### 1.1 与三个参照物的关系

| 参照物 | 它的定位 | 与 vlib 的关系 |
|---|---|---|
| librm | All-in-one **框架**（HAL + 设备 + 算法 + 测试 + 文档 + CI） | 我们要它的**工程化完整度**，不要它的**绑平台抽象方式** |
| robo-lib | 绑定到 wheel-legged 项目的**工具库** | vlib 的前身；要它的**轻**，不要它的**全局函数桩** |
| vlib | 全解耦**库** + 可选**框架脚手架** | 本次目标 |

### 1.2 一句话差异化

librm 是"给你一个装好的机器人框架"，vlib 是"给你一盒乐高，并附上几张推荐的拼装图纸"。

---

## 2. 目标与非目标

### 2.1 目标（来自需求）

| # | 目标 | 落地手段 |
|---|---|---|
| G1 | 一个仓库完成所有 | 一个 repo 内含 algo / device / module / bsp 模板 |
| G2 | 最简单接口，而非语法怪物 | 编译期鸭子类型，不用继承 + 虚函数 + 注册 |
| G3 | 全解耦库 + 可组合框架 | 设备层零 IO；框架能力以**可选**模板提供 |
| G4 | 几乎全平台 + 特定平台小优化 | 平台相关只存在于 `bsp/`；优化用 `if constexpr` |
| G5 | cmake + 直接加文件（keil）+ 自由组合 | 尽量 header-only；不强制 glob |
| G6 | 目录简单化 | 保持扁平分层目录，不嵌套 |
| G7 | 不引入其他库 | 只用 STL 子集（`std::array` 等），禁用 `vector/unordered_map` |
| G8 | 轻量化，功能小而精 | 每个模块单一职责，可单独取用 |
| G9 | 可构建 librm 式框架 | 提供 manager / scaffold 模板，但不强制 |

### 2.2 非目标（明确不做）

- **不做** all-in-one 框架：不规定任务划分、不规定 RTOS、不规定控制循环周期。
- **不做** 平台抽象基类：不出现 `CanInterface` 这种"必须继承才能用"的基类。
- **不做** 第三方库聚合：不引入 ETL、ardupilot_filter、gtest。
- **不做** 运行时多态到设备层：设备层不出现 `virtual`。

---

## 3. 设计原则

1. **依赖单向向下，且设备/算法/模块不依赖 BSP。**
   算法 ← 设备 ← 模块 是可依赖方向；`bsp/` 是叶子，只被"应用/组合层"引用。
2. **设备层零 IO。**
   设备类只做"协议编解码"，不持有总线、不发送、不接收。这样它可以被主机侧单元测试。
3. **解耦靠模板，不靠继承。**
   对总线的唯一要求是"有 `send(id, buf, len)`"，满足即插即用（编译期鸭子类型）。
4. **平台差异只在 `bsp/` 和 config 里出现。**
   `#if defined(...)` 不允许渗透到 `device/`、`algorithm/`、`module/`。
5. **接口只提供"最小充分集"。**
   宁可让用户写 5 行 adapter，也不要库给出 50 行的"通用配置对象"。
6. **一切可选。**
   BSP 是可选模板，manager 是可选脚手架，umbrella 头是可选便利。用户永远可以只 include 一个头文件。
7. **无堆分配、无全局可变状态。**
   嵌入式友好：固定大小数组、编译期确定、可重入。
8. **命名统一为 `vlib::`。** 不保留 `rb2::` 等历史命名空间。

---

## 4. 分层与依赖方向

```
┌─────────────────────────────────────────────────────────┐
│  应用层（用户代码：任务、控制循环、状态机）              │  ← 不在本库内
└─────────────────────────────────────────────────────────┘
        │  只使用
        ▼
┌─────────────────────────────────────────────────────────┐
│  vlib/module   模块：组合 device + algorithm 的纯逻辑     │  chassis / heat_limit / ...
├─────────────────────────────────────────────────────────┤
│  vlib/device   设备：纯协议编解码，零 IO，零平台依赖      │  rm_motor / dm_motor / supercap / ibus
├─────────────────────────────────────────────────────────┤
│  vlib/algorithm 算法：纯计算，零 IO，零依赖               │  pid / filter / kalman / mahony / matrix
├─────────────────────────────────────────────────────────┤
│  vlib/utils    基础：类型、概念检测、角度、CRC            │  types / concepts / angle / crc
└─────────────────────────────────────────────────────────┘
        ▲  被应用层或组合层使用（可选）
┌─────────────────────────────────────────────────────────┐
│  vlib/bsp      可选 BSP 模板：唯一平台相关处              │  stm32/*, linux/*
└─────────────────────────────────────────────────────────┘
```

**关键规则**：图中下半部分（utils → algorithm → device → module）**绝不 include `bsp/`**。
`bsp/` 是叶子节点，只被最上方的应用/组合层引用。

### 4.1 组合层（可选脚手架）的位置

"managers"（比如 `DjiMotorBus`）既不属于 device 也不属于 bsp，它是**把两者粘起来**的东西。
建议放在 `module/` 下，或单独 `scaffold/`，并标注为"可选、可替换"。

---

## 5. 核心解耦范式

这是整个库的技术核心，所有设备类都按这个模式写。

### 5.1 第一层：设备 = 纯协议编解码（零 IO）

设备类**不持有总线**、**不发送**。它只做两件事：把命令编码成字节、把字节解码成状态。

```cpp
// vlib/device/rm_motor.hpp
namespace vlib {

class RM3508 {
public:
    // 输入：物理量，纯计算
    void set_current(float amps) {
        raw_ = Clamp(amps_to_raw(amps), kMinRaw, kMaxRaw);
    }

    // 输出：把命令写进 8 字节帧的某个槽位（纯函数，无副作用）
    void encode(uint8_t (&tx)[8], uint8_t slot) const {
        tx[slot * 2]     = static_cast<uint8_t>(raw_ >> 8);
        tx[slot * 2 + 1] = static_cast<uint8_t>(raw_ & 0xFF);
    }

    // 输入：接收帧，更新状态（纯解析）
    void decode(const uint8_t rx[8]) {
        encoder_     = static_cast<int16_t>((rx[0] << 8) | rx[1]);
        velocity_    = static_cast<int16_t>((rx[2] << 8) | rx[3]);
        current_fdb_ = static_cast<int16_t>((rx[4] << 8) | rx[5]);
        temperature_ = rx[6];
    }

    // 访问器
    [[nodiscard]] float position() const { /* ... */ }
    [[nodiscard]] float velocity() const { /* ... */ }

private:
    int16_t raw_{0};
    // ... 反馈字段
};

} // namespace vlib
```

**收益**：这个类可以在 PC 上直接单元测试，不需要任何硬件、不需要任何 HAL。

### 5.2 第二层：组合层 = 编译期模板（可选脚手架）

把"一条总线 × 一组设备"聚合起来，负责**发送时机**和**反馈路由**。

```cpp
// vlib/module/dji_motor_bus.hpp  （可选脚手架）
namespace vlib {

template <class Bus, uint32_t TxId, size_t N = 8>
class DjiMotorBus {
public:
    explicit DjiMotorBus(Bus& bus) : bus_(bus) {}

    void attach(RM3508* motor, uint8_t id) {
        motors_[id] = motor;                 // id 范围 1..N
        motor->set_current(0.f);
    }

    // 上层定时调用；也可以在 CAN 中断里调用
    void flush() {
        uint8_t tx[N / 2 * 2] = {};          // 8 字节：4 个电机 × 2 字节
        for (uint8_t id = 1; id <= N; ++id) {
            if (motors_[id] != nullptr) {
                motors_[id]->encode(tx, static_cast<uint8_t>((id - 1) % 4));
            }
        }
        bus_.send(TxId, tx, sizeof(tx));     // 对 Bus 的唯一要求
    }

    // 反馈路由（由用户从中断/轮询里喂进来）
    void on_receive(uint32_t id, const uint8_t* data, size_t len) {
        const uint8_t motor_id = static_cast<uint8_t>(id - 0x200);
        if (motor_id >= 1 && motor_id <= N && motors_[motor_id] != nullptr) {
            motors_[motor_id]->decode(data);
        }
    }

private:
    Bus&     bus_;
    RM3508*  motors_[N + 1]{};
};

} // namespace vlib
```

> 注意：这里**没有**全局静态缓冲区、没有 `unordered_map`、没有 `virtual`。
> 多总线就是多个 `DjiMotorBus` 实例，互不干扰。

### 5.3 第三层：BSP = 可选的鸭子类型模板

"总线"不是某个基类，而是**任何满足下面约定的类型**：

```cpp
// 概念约定（CanBus）
//   bus.send(uint32_t id, const uint8_t* data, size_t len) -> void
// 可选（用于"特定平台小优化"，检测到才启用）：
//   bus.send_fast(...)  /  bus.is_fd()  /  bus.max_dlc()
```

用户用**自己的** HAL 满足它：

```cpp
// 用户代码：keil + STM32 HAL，6 行
struct MyCan {
    CAN_HandleTypeDef* hcan;
    void send(uint32_t id, const uint8_t* data, size_t len) {
        CAN_TxHeaderTypeDef hdr{};
        hdr.StdId = id; hdr.IDE = CAN_ID_STD;
        hdr.RTR = CAN_RTR_DATA; hdr.DLC = static_cast<uint32_t>(len);
        uint32_t mailbox = 0;
        HAL_CAN_AddTxMessage(hcan, &hdr, data, &mailbox);
    }
};

vlib::RM3508 motor;
vlib::DjiMotorBus<MyCan, 0x200> bus{my_can};
bus.attach(&motor, 1);
```

或者用 vlib 提供的模板（只是省得用户手写）：

```cpp
#include <vlib/bsp/stm32/fdcan.hpp>
vlib::bsp::FdCan<&hfdcan1> can;             // 满足 CanBus 约定
vlib::DjiMotorBus<decltype(can), 0x200> bus{can};
```

**这就是"用我的 BSP 很方便，不用我的 BSP 也能用"的精确实现。**
两种路径在编译期等价，零运行时开销。

### 5.4 概念检测：C++17 detection idiom

嵌入式环境常停留 C++17，不能用 C++20 `concept`。用 detection idiom 做一个零成本的静态检查：

```cpp
// vlib/utils/concepts.hpp
namespace vlib {

namespace detail {
template <class Bus, class = void>
struct has_send : std::false_type {};

template <class Bus>
struct has_send<Bus, std::void_t<decltype(
    std::declval<Bus&>().send(
        std::declval<uint32_t>(),
        std::declval<const uint8_t*>(),
        std::declval<size_t>()))>> : std::true_type {};
} // namespace detail

// 供 static_assert 使用
template <class Bus>
inline constexpr bool is_can_bus_v = detail::has_send<Bus>::value;

} // namespace vlib
```

使用：

```cpp
template <class Bus, uint32_t TxId>
class DjiMotorBus {
    static_assert(vlib::is_can_bus_v<Bus>,
                  "Bus 必须提供 send(uint32_t, const uint8_t*, size_t)");
    // ...
};
```

如果将来上 C++20，可以同一份代码换成真正的 `concept CanBus = requires(...)`，用户侧无感知。

### 5.5 特定平台"小优化"接口

不改变主干接口，只在**检测到**时启用更快路径：

```cpp
if constexpr (detail::has_send_fast<Bus>::value) {
    bus_.send_fast(id, data, len);   // 平台有更快路径就用
} else {
    bus_.send(id, data, len);        // 否则走通用路径
}
```

这样"几乎全平台可用"和"特定平台小优化"同时成立，且平台差异不污染设备层。

---

## 6. 目录结构

保持**扁平分层**，不嵌套、不引入 `include/` 中间层（为了 keil 直接加文件方便）。

```
vlib/
├── CMakeLists.txt          # 以 INTERFACE(header-only) 为主，可选编译单元
├── DESIGN.md               # 本文档
├── README.md
├── LICENSE
├── vlib.hpp                # umbrella 头：一键 include 全部（可选便利）
│
├── utils/                  # 基础：零依赖
│   ├── types.hpp           # u8/u16/f32 等
│   ├── concepts.hpp        # CanBus/SerialBus 等检测（新增）
│   ├── angle.hpp
│   ├── convert.hpp
│   ├── crc.hpp / crc.cpp
│   └── vmacro.h
│
├── algorithm/              # 纯算法：零 IO、零依赖
│   ├── pid.hpp
│   ├── simple_filter.hpp
│   ├── kalman_filter.hpp
│   ├── mahony.hpp / MahonyAHRS.c
│   ├── matrix.hpp
│   └── rls.hpp
│
├── device/                 # 设备：纯协议编解码，不 include 任何 HAL
│   ├── rm_motor.hpp
│   ├── dm_motor.hpp
│   ├── cubemars_motor.hpp
│   ├── supercap.hpp
│   └── ibus.hpp
│
├── module/                 # 模块：组合 device + algorithm 的纯逻辑
│   ├── chassis.hpp         # 全向/麦轮/舵轮运动学
│   ├── heat_limit.hpp
│   └── dji_motor_bus.hpp   # 可选脚手架（组合层）
│
├── bsp/                    # 可选 BSP 模板：唯一平台相关处
│   ├── stm32/
│   │   ├── fdcan.hpp
│   │   ├── bxcan.hpp
│   │   └── uart.hpp
│   └── linux/
│       └── socketcan.hpp
│
├── examples/               # 最小可跑示例（新增）
│   ├── stm32_basic/
│   └── linux_basic/
│
└── tests/                  # 主机侧单元测试（新增）
    ├── test_pid.cpp
    └── test_rm_motor.cpp
```

**关于"不嵌套 `include/`"的取舍**：嵌套 `include/vlib/...` 更规范（支持 `find_package` 安装），但 keil 加文件更麻烦。
本设计选择**扁平**，通过 CMake 的 `INTERFACE` 目标把仓库根加入 include path，使 `<vlib/xxx.hpp>` 与 `"xxx.hpp"` 都能工作（见第 7 节）。

---

## 7. 命名与 include 约定

### 7.1 命名空间

- **统一 `vlib::`**，不保留 `rb2::`。
- 子命名空间只用于真正的子领域，且不超过一层：
  - `vlib::bsp::`（BSP 模板）
  - 其余全部平铺在 `vlib::` 下（**不再区分 `vlib::comm` / `vlib::device`**，避免风格分裂）。

### 7.2 文件命名

- 统一 `snake_case.hpp`（如 `rm_motor.hpp`），C 文件用 `snake_case.c`。
- 一个文件一个主要类型，文件名 = 类型名的 snake_case。

### 7.3 include

- 库内部：`#include <vlib/utils/types.hpp>`（带前缀，避免撞名）。
- 面向用户：两种都支持——
  - `#include <vlib/vlib.hpp>`（一键全部）
  - `#include <vlib/device/rm_motor.hpp>`（按需）

### 7.4 头文件保护

统一 `#pragma once`（现有文件混用 `#ifndef` 与 `#pragma once`，需统一）。

---

## 8. 与 librm / robo-lib 的对比

| 维度 | librm | robo-lib | vlib（本文档） |
|---|---|---|---|
| 上下解耦方式 | 抽象基类 + 虚函数 + 观察者注册 + 平台宏 | 全局 C 函数桩 `BSP_CAN_Transmit()` | **模板参数 + 编译期鸭子类型** |
| 设备是否含 IO | 含（继承 `CanDevice`） | 含（调用全局桩） | **不含（纯编解码）** |
| 多态时机 | 运行时（virtual） | 链接期（全局符号） | **编译期（template）** |
| 多总线支持 | 有（`unordered_map<CanInterface*, ...>`） | 弱（全局） | **天然（每实例一份）** |
| 第三方依赖 | ETL、ardupilot_filter | 无 | **无** |
| 堆分配 | 有 | 无 | **无** |
| 可单测性 | 中（需 mock HAL） | 低（链接期桩） | **高（设备层纯函数）** |
| 开箱即用性 | 高 | 中 | 中（需选 BSP 或写 5 行 adapter） |
| 工程化完整度 | 高（测试/文档/CI） | 低 | **目标：高** |

**结论**：vlib 用"开箱即用性的少量下降"换"解耦度、轻量度、可测性的大幅提升"，并通过提供 BSP 模板把开箱即用性补回来。

---

## 9. 现状诊断（vlib 当前问题清单）

> 以下为 review 时的实测结论，供后续修复参考。**本节描述问题，不代表已经修复。**

### 9.1 构建系统

| 位置 | 问题 |
|---|---|
| `CMakeLists.txt:9` | `add_library(vlib STATIC)` 但仓库几乎全是 header，等于空库 |
| `CMakeLists.txt:59` | 引用不存在的 `include/` 目录，install 阶段会失败 |
| `CMakeLists.txt:69` | 引用不存在的 `cmake/vlibConfig.cmake.in`，配置阶段会失败 |
| `CMakeLists.txt:14` | include path 漏了 `bsp/` |
| `CMakeLists.txt:12` | 被注释的 `add_library(vlib ALIAS vlib)` 是自别名，会报错 |

### 9.2 源码（硬错误）

| 文件 | 问题 |
|---|---|
| `device/supercap.hpp:21` | 命名空间拼写为 `vilb`（应为 `vlib`） |
| `device/supercap.hpp:38-40` | **两个重名构造函数** `SuperCap(void)`，编译不过 |
| `device/supercap.hpp:17-19` | `#include "array"/"stdbool.h"/"stdint.h"` 应为 `<...>` |
| `device/supercap.hpp:54` | 成员 `source` 与局部变量 `source` 重名 |
| `module/omni-chassis.hpp:15` | 命名空间为 `rb2`，与其余文件不一致 |
| `device/supercap.hpp:22` | 使用了 `namespace device`，其余文件用 `comm`，风格分裂 |
| `device/rm-motor.hpp:36` | `static std::array txbuf`：所有实例共享、无法多总线、线程不安全 |
| `device/dm-motor.hpp:88` | `Transmit(Control)` 空实现 |
| `device/dm-motor.hpp:90` | `Receive(uint8_t*)` 空实现 |
| `device/cubemars-motor.hpp:101` | 类内 `#define PacketID_*` 泄漏宏 |
| `device/cubemars-motor.hpp:46` | `Enable/Disable/SetOrigin` 造了帧却 `(void)buf` 不发送 |

### 9.3 架构

- `bsp/can.hpp` 是空壳：`operator<<` 与 `Register` 均无实现，**没有建立任何解耦机制**。
- 设备层与传输层**完全没有接口约定**（既没有模板参数，也没有函数桩）。
- 命名空间、include 风格、头文件保护均不统一。

### 9.4 完成度雷达

| 层 | 现状 | 完成度 |
|---|---|---|
| algorithm | pid/kalman/mahony/rls/matrix/filter 基本齐 | ~60% |
| device | 有协议壳，但无 IO 接线、风格不一、有编译错 | ~25% |
| module | 只有空壳 chassis / heat-limit | ~5% |
| bsp | 只有一个空的 can.hpp | ~5% |
| 工程化 | CMake 坏、无测试/文档/示例/CI | ~10% |
| **整体** | | **~15–20%** |

---

## 10. 路线图

### P0 —— 让它先"是一个库"（不涉及新功能）

1. 重写 `CMakeLists.txt`：INTERFACE(header-only) 目标、统一 include root、`bsp/` 入 path、可选 install/export。
2. 修复第 9.2 节全部编译错误。
3. 统一命名空间为 `vlib::`、统一 `#pragma once`、统一文件命名。
4. 新增 umbrella 头 `vlib.hpp`。

**验收**：`cmake -B build && cmake --build build` 在 Linux 主机通过；keil 侧只加 include path 即可编译一个只 include 算法层的文件。

### P1 —— 建立"上下解耦"核心（本设计的灵魂）

5. `utils/concepts.hpp`：实现 `is_can_bus_v` / `is_serial_bus_v` 等检测。
6. 设备层改造为"纯编解码 + Bus 注入"或纯编解码 + 组合层；移除 `static txbuf`。
7. `bsp/` 落地：STM32 FDCAN/BXCAN、UART、Linux socketcan 各一个 adapter（每个约 30 行）。
8. `module/` 补齐：底盘运动学（mecanum/omni/swerve）、功率限制、AHRS 封装、`DjiMotorBus` 脚手架。

**验收**：能用 `examples/linux_basic` 在 PC 上跑通"构造电机 → 编码 → 用 socketcan 发送"的完整链路；设备层可单测。

### P2 —— 对齐 librm 的工程化完整度

9. `tests/`：CMake + CTest，纯算法/协议层用 assert 即可。
10. 文档：README + 每个头文件顶部注释 + 至少一篇"如何接线（STM32）"教程。
11. `examples/`：stm32 与 linux 两个最小可跑示例。
12. CI：GitHub Actions，至少 Linux 主机编译 + arm-none-eabi 交叉编译。

### P3 —— 可选增强

13. `ARM_MATH` 软硬件浮点开关。
14. 轻量平台检测脚本（借鉴 librm `cmake/detect_platform.cmake`，但做成可选引入）。

---

## 11. 待决策事项

| # | 事项 | 本设计倾向 | 状态 |
|---|---|---|---|
| D1 | 命名空间 | 统一 `vlib::`，不保留 `rb2::` | **已定** |
| D2 | include 前缀 | 库内带 `vlib/` 前缀；用户侧两种都支持 | 待确认 |
| D3 | 是否 header-only 为主 | 是（换取 keil 直接加文件） | 待确认 |
| D4 | BSP 模板放 `bsp/` 还是随设备放 | 独立 `bsp/`（平台相关集中） | 待确认 |
| D5 | 组合层（manager）放 `module/` 还是独立 `scaffold/` | 先放 `module/`，标注为可选 | 待确认 |

---

## 附录 A：一个完整的"接线"示例（目标形态）

```cpp
// 用户的应用代码（不强制任何结构，这里只是最朴素的一种）
#include <vlib/vlib.hpp>
#include <vlib/bsp/stm32/fdcan.hpp>

extern FDCAN_HandleTypeDef hfdcan1;

static vlib::bsp::FdCan<&hfdcan1> g_can{};
static vlib::DjiMotorBus<decltype(g_can), 0x200> g_chassis_bus{g_can};
static vlib::RM3508 g_wheel[5];
static vlib::PID   g_wheel_pid[5];

extern "C" void App_Init() {
    for (uint8_t i = 1; i <= 4; ++i) {
        g_chassis_bus.attach(&g_wheel[i], i);
        g_wheel_pid[i].SetGains(6.f, 0.f, 0.2f);
    }
}

// 1kHz 控制回调
extern "C" void App_Loop() {
    for (uint8_t i = 1; i <= 4; ++i) {
        const float out = g_wheel_pid[i].Update(g_wheel[i].velocity(), target_[i]);
        g_wheel[i].set_current(out);
    }
    g_chassis_bus.flush();          // 唯一的"发送"动作
}

// 用户自己的 CAN 中断里
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* h, uint32_t bits) {
    FDCAN_RxHeaderTypeDef hdr; uint8_t data[8];
    HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, data);
    g_chassis_bus.on_receive(hdr.Identifier, data, hdr.DataLength);
}
```

**这个例子里，vlib 没有规定任何东西**：任务结构是用户的、中断是用户的、周期是用户的。
vlib 只提供了 `RM3508`、`PID`、`FdCan`、`DjiMotorBus` 四块积木。
