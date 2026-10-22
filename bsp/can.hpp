/**
 * @file can.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-10-23
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace vlib {

struct Device_Config {
    using Backend  = void*;
    using Callback = void*;
    using Transmit = void*;
    using Register = void*;
    using Init     = void*;
};

template<class CONFIG>
struct Device_API {
    struct Frame {
        uint32_t             id = 0;
        std::vector<uint8_t> data {};
    };

    using Backend  = typename CONFIG::Backend;
    using Callback = typename CONFIG::Callback;

    explicit CAN_API(Backend h):
        backend(h) {}

    void Init() { CONFIG::Init(backend); }

    bool Transmit(const CAN_Frame& frame) { return CONFIG::Transmit(backend, frame); }
    void RegisterCallback(uint32_t id, Callback cb) { CONFIG::Register(backend, id, callback); }

    // 语法糖，方便使用
    CAN& operator<<(const CAN_Frame& frame) {
        Transmit(frame);
        return *this;
    }

private:
    Backend backend;
};

} // namespace vlib
