/**
 * @file thread.hpp
 * @brief 统一的线程/任务抽象（后端可插拔，零平台依赖）
 *
 * 提供一种"创建进程/任务"的统一方法：
 *   - STM32 -> FreeRTOS 任务（vlib/bsp/stm32/freertos_thread.hpp）
 *   - 主机   -> std::thread  （vlib/bsp/linux/std_thread.hpp）
 *
 * 本文件不 include 任何平台头文件。
 */

#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>

namespace vlib {

struct ThreadSpec {
    const char* name     = "task";
    uint32_t    stack    = 512; // 栈大小（word，1 word = 4 字节）
    int         priority = 0;
};

template<class Backend>
class Thread {
public:
    using Entry = void (*)(void*);

    Thread() = default;
    ~Thread() { backend_.destroy(); }

    Thread(const Thread&)            = delete;
    Thread& operator=(const Thread&) = delete;

    bool Start(Entry fn, void* arg = nullptr, const ThreadSpec& spec = ThreadSpec {}) { return backend_.create(fn, arg, spec); }

    void Stop() { backend_.destroy(); }
    bool Join() { return backend_.join(); }
    bool Running() const { return backend_.started(); }

private:
    Backend backend_ {};
};

} // namespace vlib
