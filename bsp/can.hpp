/**
 * @file can.hpp
 * @brief 通用 CAN 接口（零平台依赖）
 *
 * 描述符 CONFIG 由 BSP 提供，必须实现：
 *   using Backend  = ...;                              // 平台句柄（如 void*）
 *   using Frame    = ...;                              // 帧类型
 *   using Callback = void (*)(const Frame&);           // 接收回调
 *   static void init(Backend);
 *   static bool transmit(Backend, const Frame&);
 *   static void subscribe(Backend, uint32_t id, Callback);
 */

#pragma once

#include <array>
#include <stddef.h>
#include <stdint.h>

namespace vlib {

/// 通用 CAN 帧：LEN = 8（经典 CAN）/ 64（CAN FD）
template<size_t LEN = 8>
struct CAN_Frame {
    uint32_t                 id = 0;
    std::array<uint8_t, LEN> data {};
};

namespace device {

struct Config {};

template<typename CONFIG>
class Interface {
    using Backend  = typename CONFIG::Backend;
    using Frame    = typename CONFIG::Frame;
    using Callback = typename CONFIG::Callback;
    Backend backend_;

public:
    explicit Interface(Backend h):
        backend_(h) {}
    void       Init() { CONFIG::init(backend_); }
    bool       Transmit(const Frame& frame) { return CONFIG::transmit(backend_, frame); }
    Interface& operator<<(const Frame& frame) {
        Transmit(frame);
        return *this;
    }
    void RegisterCallback(uint32_t id, Callback cb) { CONFIG::subscribe(backend_, id, cb); }
};

} // namespace device
} // namespace vlib
