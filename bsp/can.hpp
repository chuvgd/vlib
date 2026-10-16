/**
 * @file can.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 用于提供一种bsp框架形式，若使用配合vlib会很好，不用也可以单独使用其他模块
 * @version 0.1.0
 * @date 2026-10-13
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#pragma once

#include <array>
#include <cstdint>

namespace vlib {

class CAN {
public:
    CAN() {}

    template<size_t LEN>
    struct Massage {
        uint32_t                 id;
        std::array<uint8_t, LEN> buf;
    };

    template<size_t LEN>
    struct CallbackHandle {
        uint32_t id;
        void (*func)(std::array<uint8_t, LEN> buf);
    };

    template<size_t LEN>
    CAN& operator<<(Massage<LEN>& msg) {
        // static can msg head = {.DLC = LEN, .id = msg.id}
        // can add massage(msg.buf)
        return *this;
    }

    template<size_t LEN>
    void Register(CallbackHandle<LEN>& cb) {
        // can register function
        //
    }
};

} // namespace vlib
